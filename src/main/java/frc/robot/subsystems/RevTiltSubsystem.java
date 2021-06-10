package frc.robot.subsystems;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {

    public final int POSITION_SLOT = 0;
    public final int SMART_MOTION_SLOT = 1;
    public final int VISION_SLOT = 2;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;
    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    public final CANPIDController mPidController;
    public final PIDController tiltLockController = new PIDController(.032, 0.001, 0);
    public CANDigitalInput m_reverseLimit;
    private ISimWrapper mElevatorSim;
    public boolean positionResetDone;
    public double targetAngle;
    private double inPositionBandwidth = .2;
    public double targetVerticalOffset;
    public double driverVerticalOffset;
    public boolean validTargetSeen;
    public double adjustedTargetAngle;
    private final static double pivotDistance = 10.5;// inches
    private final double[] pinDistances = { 2.1, 3.0842519685, 4.068503937, 5.0527559055, 6.037007874, 7.0212598425,
            8.005511811 };
    public final double cameraBaseAngle = HoodedShooterConstants.TILT_MIN_ANGLE;
    public final double tiltMaxAngle = HoodedShooterConstants.TILT_MAX_ANGLE;

    public final double degreesPerRev = HoodedShooterConstants.tiltDegreesPerRev;// degrees per motor turn
    public final double tiltMinAngle = HoodedShooterConstants.TILT_MIN_ANGLE;

    public boolean tuneOn = false;
    public boolean lastTuneOn;

    public boolean lockTuneOn = false;
    public boolean lastLockTuneOn;

    public boolean tiltMotorConnected;
    private double maxAdjustShoot = .45;
    public double motorEndpointDegrees;
    public int faultSeen;
    public double lockPIDOut;
    public boolean visionOnTarget;

    /**
     * 
     */

    public RevTiltSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(false);
        m_motor.setOpenLoopRampRate(5);
        aimCenter();
        mEncoder.setPosition(0);
        targetAngle = tiltMaxAngle;

        if (RobotBase.isReal()) {

            mEncoder.setPositionConversionFactor(degreesPerRev);
            mEncoder.setVelocityConversionFactor(degreesPerRev / 60);

        }

        else {
            mEncoder.setPositionConversionFactor(degreesPerRev * 3);
            mEncoder.setVelocityConversionFactor(degreesPerRev * 3 / 60);

        }

        setTiltLockGains();
        resetAngle();
        m_motor.setIdleMode(IdleMode.kBrake);

        m_reverseLimit = m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);

        m_reverseLimit.enableLimitSwitch(RobotBase.isReal());

        if (RobotBase.isReal() && m_reverseLimit.get()) {
            resetAngle();
        }

        setSoftwareLimits();

        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = .001;
            ElevatorSimConstants.kElevatorGearing = 1;
            ElevatorSimConstants.kMaxElevatorHeight = 100;// HoodedShooterConstants.TILT_MAX_ANGLE;
            ElevatorSimConstants.kMinElevatorHeight = -100;// HoodedShooterConstants.TILT_MIN_ANGLE;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));

            mPidController.setP(.12);

        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkTune();

        if (RobotBase.isReal() && DriverStation.getInstance().isDisabled())
            targetAngle = getAngle();

        // SmartDashboard.putNumber("CTA", calculateTiltAngle());

        if (faultSeen != 0)
            faultSeen = getFaults();

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

        mPidController.setReference(motorDegrees, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public boolean lockTiltToVision(double cameraError) {
        lockPIDOut = tiltLockController.calculate(cameraError, 0);
        m_motor.set(lockPIDOut);
        targetAngle = getAngle();
        return tiltLockController.atSetpoint();
    }

    public void resetAngle() {
        mEncoder.setPosition(0);
        targetAngle = tiltMaxAngle;
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
        return tiltMaxAngle - getMotorDegrees();
    }

    public double getCameraAngle() {
        return getAngle();
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
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean onMinusHardwarLimit() {
        return m_motor.getFault(FaultID.kHardLimitRev);
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
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kReverse, (float) 0.);
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kForward, (float) (29));
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSoftLimits(boolean on) {
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    @Override
    public boolean isAtHeight(double inches) {
        // TODO Auto-generated method stub
        return ElevatorSubsystem.super.isAtHeight(inches);
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
        faultSeen = 0;
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void aimLower(double angle) {
        if (driverVerticalOffset < maxAdjustShoot)
            driverVerticalOffset += angle;
    }

    public void aimHigher(double angle) {
        if (driverVerticalOffset > -maxAdjustShoot)
            driverVerticalOffset -= angle;
    }

    public void aimCenter() {
        driverVerticalOffset = 0;
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
        // SmartDashboard.putNumber("PInDist", pinDistance);
        double valRads = 2 * Math.asin(pinDistance / (2 * pivotDistance));
        return cameraBaseAngle + Math.toDegrees(valRads);
    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = mPidController.getP(slotNumber);

        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = mPidController.getI(slotNumber);
        }

        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = mPidController.getD(slotNumber);
        }
        if (f != lastkFF) {
            mPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIz = mPidController.getIZone(slotNumber);
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mPidController.setSmartMotionMaxVelocity(maxAcc, slotNumber);
                lastmaxVel = maxVel;
            }

            if (lastallowedErr != allowedErr) {
                mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);
                lastallowedErr = allowedErr;
            }
        }
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
        kFF = .0004;// 10000 rpm = 10000 * .25 deg per rev= 2500 1/2500 = .0004
        kMaxOutput = .5;
        kMinOutput = -.5;
        maxRPM = 11000;// not used
        allowedErr = .01;
    }

    private void setTiltLockGains() {

        tiltLockController.setP(Pref.getPref("TiLkP"));
        tiltLockController.setI(Pref.getPref("TiLkI"));
        tiltLockController.setD(Pref.getPref("TiLkD"));
        double Izone = Pref.getPref("TiLkIZ");
        tiltLockController.setIntegratorRange(-Izone, Izone);
        tiltLockController.setTolerance(.1);
    }

    private void checkTune() {

        CANError burnError = CANError.kError;
        SmartDashboard.putBoolean("TiltBurnOK", false);
        if (Pref.getPref("tITune") == 5.) {
            burnError = m_motor.burnFlash();
            SmartDashboard.putBoolean("TiltBurnOK", burnError == CANError.kOk);
        }

        tuneOn = Pref.getPref("tITune") == 1.;

        if (tuneOn && !lastTuneOn) {

            tuneGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

        // Lock controller

        lockTuneOn = Pref.getPref("tiLTune") != 0.;

        if (lockTuneOn && !lastLockTuneOn) {

            setTiltLockGains();
            lastLockTuneOn = true;
        }

        if (lastLockTuneOn)
            lastLockTuneOn = lockTuneOn;

    }

}
