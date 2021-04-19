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
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.sim.ElevatorSubsystem;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {
    private static final double GRAVITY_COMPENSATION_VOLTS = .001;
    private static final double DEG_PER_ENCODER_REV = .00029;
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
    public boolean tuneOn;

    public RevTiltSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setOpenLoopRampRate(5);
        if (RobotBase.isReal())
            mEncoder.setPositionConversionFactor(DEG_PER_ENCODER_REV); // HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV);
        else
            mEncoder.setPositionConversionFactor(1); //
        mEncoder.setPosition(0);

        m_reverseLimit = m_motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(RobotBase.isReal());
        if (m_reverseLimit.get()) {
            resetAngle(0);
        }
        if (tuneOn)
            setGains();

        targetAngle = getAngle();
        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = 2;
            ElevatorSimConstants.kElevatorGearing = 10;
            ElevatorSimConstants.kMaxElevatorHeight = 10;
            ElevatorSimConstants.kMinElevatorHeight = -1;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (tuneOn)
            tuneGains();

    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void moveManually(double speed) {
        targetAngle = getAngle();
        m_motor.set(speed);
    }

    @Override
    public void goToPosition(double degrees) {
        mPidController.setReference(degrees, ControlType.kPosition, POSITION_SLOT, GRAVITY_COMPENSATION_VOLTS,
                CANPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void goToPositionMotionMagic(double degrees) {
        mPidController.setReference(degrees, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public void resetAngle(double position) {
        mEncoder.setPosition(position);
    }

    @Override
    public boolean isAtHeight(double inches, double allowableError) {
        return Math.abs(inches - getHeightInches()) < allowableError;
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    @Override
    public double getHeightInches() {
        return Units.metersToInches(mEncoder.getPosition());
    }

    public double getAngle() {
        return mEncoder.getPosition();
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
                (float) HoodedShooterConstants.TILT_MAX_ANGLE);
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

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz) {
        mPidController.setIAccum(0);
        mPidController.setP(p);
        mPidController.setI(i);
        mPidController.setD(d);
        mPidController.setFF(f);
        mPidController.setIZone(kIz);
    }

    private void setGains() {
        // PID coefficients
        kP = 5e-1;
        kI = 0;// 1e-5;
        kD = 0;
        kIz = 1;
        kFF = 0.0000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 11000;// not used
        allowedErr = 1;
        // Smart Motion Coefficients
        maxVel = 10000; // rpm
        maxAcc = 75;

        // set PID coefficients

        calibratePID(kP, kI, kD, kFF, kIz);

        mPidController.setP(kP, SMART_MOTION_SLOT);
        mPidController.setI(kI, SMART_MOTION_SLOT);
        mPidController.setD(kD, SMART_MOTION_SLOT);
        mPidController.setIZone(kIz, SMART_MOTION_SLOT);
        mPidController.setFF(kFF, SMART_MOTION_SLOT);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

    private void tuneGains() {

        double p = Robot.tuneValues.getEntry("kP").getDouble(0.002);
        double i = Robot.tuneValues.getEntry("kI").getDouble(0.01);
        double d = Robot.tuneValues.getEntry("kD").getDouble(0);
        double ff = Robot.tuneValues.getEntry("kFF").getDouble(2);
        double iz = Robot.tuneValues.getEntry("kIZ").getDouble(2e-4);

        calibratePID(p, i, d, ff, iz);
    }

}
