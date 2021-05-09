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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {

    public final int POSITION_SLOT = 0;
    public final int VISION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kAcc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastkAcc;

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

    public final double degreesPerRev = HoodedShooterConstants.tiltDegreesPerRev;// degrees per motor turn
    public final double tiltMinAngle = HoodedShooterConstants.TILT_MIN_ANGLE;
    public boolean tuneOn = false;
    private int loopCtr;
    public boolean tiltMotorConnected;
    public boolean lastTuneOn;
    private double startTime;
    private double endTime;

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
        mEncoder.setPosition(0);
        targetAngle = tiltMinAngle;

        if (RobotBase.isReal()) {

            mEncoder.setPositionConversionFactor(degreesPerRev);
            mEncoder.setVelocityConversionFactor(degreesPerRev / 60);

        }

        else {
            mEncoder.setPositionConversionFactor(degreesPerRev * 3);
            mEncoder.setVelocityConversionFactor(degreesPerRev * 3 / 60);

        }

        setGains();
        mEncoder.setPosition(0);
        resetAngle(0);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.setSmartCurrentLimit(10, 10);
        m_reverseLimit = m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(false);
        if (m_reverseLimit.get()) {
            resetAngle(0);
        }

        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = .001;
            ElevatorSimConstants.kElevatorGearing = 1;
            ElevatorSimConstants.kMaxElevatorHeight = 30;// HoodedShooterConstants.TILT_MAX_ANGLE;
            ElevatorSimConstants.kMinElevatorHeight = 0;// HoodedShooterConstants.TILT_MIN_ANGLE;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));

            mPidController.setP(.1);
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        loopCtr++;

        loopCtr++;

        tuneOn = Pref.getPref("tILTune") != 0.;

        if (tuneOn && !lastTuneOn) {
            startTime = Timer.getFPGATimestamp();
            tuneGains();
            lastTuneOn = true;
            endTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("TGT", endTime - startTime);
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

        if (RobotBase.isReal() && DriverStation.getInstance().isDisabled())
            targetAngle = getAngle();

    }

    public boolean checkCAN() {
        return tiltMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    public double getIaccum() {
        return mPidController.getIAccum();
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
        SmartDashboard.putNumber("TIGET", m_motor.getAppliedOutput());
    }

    @Override
    public void goToPosition(double motorTurns) {
        mPidController.setReference(motorTurns, ControlType.kPosition, POSITION_SLOT);

    }

    public void positionTilt(double motorDegrees, int slotNumber) {
        mPidController.setReference(motorDegrees, ControlType.kPosition, slotNumber);

    }

    @Override
    public void goToPositionMotionMagic(double motorDegrees) {

        // convert angle to motor turns

        mPidController.setReference(motorDegrees, ControlType.kSmartMotion, POSITION_SLOT);
    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
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

    public double getMotorDegrees() {
        return mEncoder.getPosition();
    }

    public double getAngle() {
        return tiltMinAngle + getMotorDegrees();
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
        // m_motor.updateSim();
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
        return 0;// m_motor.getFaults();
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

        int lsTurns = (int) getMotorDegrees() / 20;
        double rem = getMotorDegrees() / 40 - (double) lsTurns;

        // get angle from looking up pin distance table and interpolating for
        // remainder
        double pinDistance = pinDistances[lsTurns] + (pinDistances[lsTurns + 1] - pinDistances[lsTurns]) * rem;
        SmartDashboard.putNumber("PInDist", pinDistance);
        double valRads = 2 * Math.asin(pinDistance / (2 * pivotDistance));
        return cameraBaseAngle + Math.toDegrees(valRads);
    }

    public void calibratePID(double p, double i, double d, double acc, double kIz, int slotNumber) {

        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = p;// mPidController.getP(slotNumber);

        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = i;// mPidController.getI(slotNumber);
        }

        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = d;// mPidController.getD(slotNumber);
        }
        if (acc != lastkAcc) {
            m_motor.setClosedLoopRampRate(acc);
            lastkAcc = acc;// m_motor.getClosedLoopRampRate();
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);

            lastkIz = kIz;// mPidController.getIZone(slotNumber);
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }

    }

    private void setGains() {
        /**
         * PID coefficients//max rpm = 11000 and degrees per rev = 1.421
         * 
         * 
         * 11000 * 1.421 degrees per rev = 15631 deg per minute
         * 
         * 100% FF = 1/15631 = 6.9 e-5
         * 
         * 90% FF = 5.8 e-5 15631 deg per min = 250 degrees per second = .8 seconds full
         * travel +100 to -100 degrees
         */
        fixedSettings();

        kP = .0006;
        kI = 0.000012;
        kD = .000001;
        kIz = 1;

        kAcc = 500;

        // set PID coefficients
        calibratePID(kP, kI, kD, kAcc, kIz, POSITION_SLOT);
        calibratePID(kP, kI, kD, kAcc, kIz, VISION_SLOT);

    }

    private void tuneGains() {
        fixedSettings();

        double p = Pref.getPref("tILKp");
        double i = Pref.getPref("tILKi");
        double d = Pref.getPref("tILKd");
        double iz = Pref.getPref("tILKiz");
        kAcc = Pref.getPref("tILAcc");

        calibratePID(p, i, d, kAcc, iz, POSITION_SLOT);
        calibratePID(p, i, d, kAcc, iz, VISION_SLOT);
    }

    private void fixedSettings() {
        kFF = .0000;//
        mPidController.setFF(kFF);
        kMaxOutput = 1;
        kMinOutput = -1;

    }

}
