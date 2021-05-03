package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTurretSubsystem extends SubsystemBase implements ElevatorSubsystem {

    private static final double DEG_PER_MOTOR_REV = HoodedShooterConstants.TURRET_DEG_PER_MOTOR_REV;
    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mElevatorSim;
    public double visionCorrection;
    public double targetAngle;
    private double inPositionBandwidth = 1;
    public double targetHorizontalOffset;
    public boolean validTargetSeen;
    public double adjustedTargetAngle;

    public boolean tuneOn = false;

    public RevTurretSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setOpenLoopRampRate(5);
        mEncoder.setPosition(0);
        aimCenter();
        if (!tuneOn)
            setGains();

        int stallLimit = 5;
        int freeLimit = 5;
        int limitRPM = 0;
        if (RobotBase.isReal()) {
            m_motor.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

            mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);// 1 /
            mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);
        } else

        {
            mEncoder.setPositionConversionFactor(1); // // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV);
            mPidController.setP(.1, SMART_MOTION_SLOT);
            // mPidController.setFF(0.000005, SMART_MOTION_SLOT);
        }

        m_motor.setIdleMode(IdleMode.kBrake);

        // if (RobotBase.isSimulation())
        // mPidController.setP(0.16);

        mEncoder.setPosition(0);
        targetAngle = 0;

        setSoftwareLimits();

        if (RobotBase.isSimulation())

        {
            ElevatorSimConstants.kCarriageMass = .001;
            ElevatorSimConstants.kElevatorGearing = 1;
            ElevatorSimConstants.kMaxElevatorHeight = 100;
            ElevatorSimConstants.kMinElevatorHeight = -110;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));

        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        tuneOn = Pref.getPref("tUTune") != 0.;
        if (tuneOn)
            tuneGains();

    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void moveManually(double speed) {
        m_motor.set(speed);
        targetAngle = getAngle();
    }

    @Override
    public void goToPosition(double angle) {
        mPidController.setReference(angle, ControlType.kPosition, POSITION_SLOT);
    }

    @Override
    public void goToPositionMotionMagic(double angle) {

        mPidController.setReference(angle, ControlType.kSmartMotion, SMART_MOTION_SLOT);

    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
        mPidController.setIAccum(0);
        targetAngle = angle;
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

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kForward,
                (float) HoodedShooterConstants.TURRET_MAX_ANGLE);
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kReverse,
                (float) HoodedShooterConstants.TURRET_MIN_ANGLE);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    @Override
    public boolean isAtHeight(double angle, double allowableError) {
        return Math.abs(angle - getAngle()) < allowableError;
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    public double getAngle() {
        return mEncoder.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        mElevatorSim.update();
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    public void moveManuallyVelocity(double speed) {
        targetAngle = getAngle();
        mPidController.setReference(speed, ControlType.kVelocity, POSITION_SLOT);
        // SmartDashboard.putNumber("PSGetP", mPidController.getP(POSITION_SLOT));
        // SmartDashboard.putNumber("PS1FF", mPidController.getFF(POSITION_SLOT));
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
        /**
         * PID coefficients//max rpm = 11000 and degrees per rev = .295 11000 rpm =
         * 11000 * .295 degrees per rev = 3245 deg per minute 100% FF = 1/3245 = 3.08
         * e-4 90% FF = .9 * 3.08 e-4 = 2.77 e-4 3245 deg per min = 54 degrees per
         * second = 4 seconds full travel +100 to -100 degrees
         */
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
        double p = Pref.getPref("tUKp");
        double i = Pref.getPref("tUki");
        double d = Pref.getPref("tUKd");
        double iz = Pref.getPref("tUKiz");
        maxVel = Pref.getPref("tUMaxV");
        maxAcc = Pref.getPref("tUMaxA");

        calibratePID(p, i, d, kFF, iz, SMART_MOTION_SLOT);
    }

    private void fixedSettings() {
        kFF = .000078;//
        kMaxOutput = .5;
        kMinOutput = -.5;
        maxRPM = 11000;// not used
        allowedErr = 1;

    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void aimFurtherLeft(double angle) {
        targetHorizontalOffset -= angle;
    }

    public void aimFurtherRight(double angle) {
        targetHorizontalOffset += angle;
    }

    public void aimCenter() {
        targetHorizontalOffset = 0;
    }

}
