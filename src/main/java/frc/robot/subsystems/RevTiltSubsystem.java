package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {

    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    public CANDigitalInput m_reverseLimit;
    private ISimWrapper mElevatorSim;
    public double visionCorrection;
    public boolean positionResetDone;
    public double targetAngle;
    private double inPositionBandwidth = 1;

    public double targetVerticalOffset;
    public boolean validTargetSeen;
    public double adjustedTargetAngle;
    private final static double pivotDistance = 10.5;// inches
    private final double[] pinDistances = { 2.1, 3.0842519685, 4.068503937, 5.0527559055, 6.037007874, 7.0212598425,
            8.005511811 };
    private final double cameraBaseAngle = HoodedShooterConstants.TILT_MIN_ANGLE;
    private final double leadscrewAngleSlope = HoodedShooterConstants.leadscrewAngleSlope;
    private final double motorAngleSlope = HoodedShooterConstants.motorAngleSlope;// degrees per motor turn
    private final static double tiltMinAngle = HoodedShooterConstants.TILT_MIN_ANGLE;
    public boolean tuneOn = false;
    private int loopCtr;
    public boolean tiltMotorConnected;

    /**
     * 
     */

    public RevTiltSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setOpenLoopRampRate(5);
        aimCenter();

        if (!tuneOn)
            setGains();
        if (RobotBase.isSimulation()) {
            mPidController.setP(.1, SMART_MOTION_SLOT);
            mPidController.setFF(0.000008, SMART_MOTION_SLOT);
            setSoftwareLimits();
        }

        resetAngle();
        m_motor.setIdleMode(IdleMode.kBrake);

       

        m_motor.setSmartCurrentLimit(5);
        m_reverseLimit = m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        m_reverseLimit.enableLimitSwitch(true);
        if (m_reverseLimit.get()) {
            resetAngle();
        }
        setSoftwareLimits();
        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = .001;
            ElevatorSimConstants.kElevatorGearing = 1;
            ElevatorSimConstants.kMaxElevatorHeight = HoodedShooterConstants.TILT_MAX_ANGLE;
            ElevatorSimConstants.kMinElevatorHeight = HoodedShooterConstants.TILT_MIN_ANGLE;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        loopCtr++;
        if (loopCtr > 24) {

            tuneOn = Pref.getPref("tITune") != 0.;
            if (tuneOn)
                tuneGains();

            tiltMotorConnected = m_motor.getFirmwareVersion() != 0;
            loopCtr = 0;
        }

    }

    @Override
    public void close() {
        m_motor.close();
    }

    public void moveManuallyVelocity(double speed) {
        targetAngle = getAngle();
        mPidController.setReference(speed, ControlType.kVelocity);
    }

    @Override
    public void moveManually(double speed) {
        targetAngle = getAngle();
        m_motor.set(speed);
    }

    @Override
    public void goToPosition(double angle) {
        // convert angle to motor turns
        double motorTurns = angle / motorAngleSlope;// deg /deg per turn
        mPidController.setReference(motorTurns, ControlType.kPosition, POSITION_SLOT);
    }

    @Override
    public void goToPositionMotionMagic(double angle) {
        // SmartDashboard.putNumber("TISMMA",
        // mPidController.getSmartMotionMaxAccel(SMART_MOTION_SLOT));
        // SmartDashboard.putNumber("TISMMV",
        // mPidController.getSmartMotionMaxVelocity(SMART_MOTION_SLOT));
        // SmartDashboard.putNumber("TISMMP", mPidController.getP(SMART_MOTION_SLOT));
        // SmartDashboard.putNumber("TISMMF", mPidController.getFF(SMART_MOTION_SLOT));

        // SmartDashboard.putNumber("TISMMI", mPidController.getI(SMART_MOTION_SLOT));
        // SmartDashboard.putNumber("TISMIZ",
        // mPidController.getIZone(SMART_MOTION_SLOT));

        // convert angle to motor turns
        double motorTurns = (angle - tiltMinAngle) / motorAngleSlope;// deg /deg per turn

        // SmartDashboard.putNumber("MotorTurns", motorTurns);

        mPidController.setReference(motorTurns, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public void resetAngle() {
        mEncoder.setPosition(0);
        targetAngle = tiltMinAngle;
        mPidController.setIAccum(0);
    }

    @Override
    public boolean isAtHeight(double inches, double allowableError) {
        return Math.abs(inches - getAngle()) < allowableError;
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    public double getMotorTurns() {
        return mEncoder.getPosition();
    }

    public double getLeadscrewTurns() {
        return mEncoder.getPosition() / 20;
    }

    public double getLeadscrewAngle() {
        return getLeadscrewTurns() * leadscrewAngleSlope;
    }

    public double getAngle() {
        return tiltMinAngle + getLeadscrewAngle();
    }

    public double getOut() {
        return m_motor.get();
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getSpeed() {
        return mEncoder.getVelocity();
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    @Override
    public void simulationPeriodic() {
        mElevatorSim.update();
        m_motor.updateSim();
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kForward,
                (float) HoodedShooterConstants.maxMotorTurns);
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void aimHigher(double angle) {
        targetVerticalOffset += angle;
    }

    public void aimLower(double angle) {
        targetVerticalOffset -= angle;
    }

    public void aimCenter() {
        targetVerticalOffset = 0;
    }

    /**
     * The angle should be goverened by angle =(2*(ASIN(distance between
     * pins/2)/10.5)) this will be in radians.
     * 
     * 
     * @return
     */
    public double calculateTiltAngle() {
        // separate angle into integer and remainder

        int lsTurns = (int) getLeadscrewTurns();
        double rem = getLeadscrewTurns() / 2 - (double) lsTurns;

        // get angle from looking up pin distance table and interpolating for remainder
        double pinDistance = pinDistances[lsTurns] + (pinDistances[lsTurns + 1] - pinDistances[lsTurns]) * rem;
        SmartDashboard.putNumber("PInDist", pinDistance);
        double valRads = 2 * Math.asin(pinDistance / (2 * pivotDistance));
        return cameraBaseAngle + Math.toDegrees(valRads);
    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        mPidController.setIAccum(0);
        mPidController.setP(p, slotNumber);
        mPidController.setI(i, slotNumber);
        mPidController.setD(d, slotNumber);
        mPidController.setFF(f, slotNumber);
        mPidController.setIZone(kIz);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
        mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);

    }

    private void setGains() {

        fixedSettings();
        kP = .000001;
        kI = 0;
        kD = .0005;
        kIz = 0;
        maxVel = 5000; // motor rev per min
        maxAcc = 7500;

        // set PID coefficients

        calibratePID(kP, kI, kD, kFF, kIz, SMART_MOTION_SLOT);

    }

    private void tuneGains() {

        fixedSettings();

        double p = Pref.getPref("tIKp");
        double i = Pref.getPref("tIKi");
        double d = Pref.getPref("tIKd");
        double iz = Pref.getPref("tIKiz");
        maxVel = Pref.getPref("tIMaxV");
        maxAcc = Pref.getPref("tIMaxA");

        calibratePID(p, i, d, kFF, iz, SMART_MOTION_SLOT);
    }

    private void fixedSettings() {
        kFF = .000078;//
        kMaxOutput = .5;
        kMinOutput = -.5;
        maxRPM = 11000;// not used
        allowedErr = 1;

    }

}
