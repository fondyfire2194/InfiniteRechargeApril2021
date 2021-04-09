package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.ElevatorSubsystem;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {
    private static final double GRAVITY_COMPENSATION_VOLTS = .5;
    private static final double TICKS_PER_DEGREE = 3448;
    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    public DigitalInput m_reverseLimit = new DigitalInput(9);
    private ISimWrapper mElevatorSim;
    public double visionCorrection;
    public boolean positionResetDone;
    public double targetAngle;
    private double inPositionBandwidth = 1;

    public RevTiltSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();

        mEncoder.setPositionConversionFactor(TICKS_PER_DEGREE); // HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV);
        mEncoder.setPosition(0);
        mPidController.setP(0.16);
        m_motor.restoreFactoryDefaults();

        gainSettings();
        targetAngle = getAngle();
        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = 2;
            ElevatorSimConstants.kElevatorGearing = 18;
            ElevatorSimConstants.kMaxElevatorHeight = 10;
            ElevatorSimConstants.kMinElevatorHeight = -1;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNEO(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void moveManually(double speed) {
        m_motor.set(speed);
    }

    @Override
    public void goToPosition(double degrees) {
        mPidController.setReference(degrees, ControlType.kPosition, POSITION_SLOT, GRAVITY_COMPENSATION_VOLTS,
                CANPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void goToPositionMotionMagic(double degrees) {
        SmartDashboard.putNumber("MMTILA", degrees);
        SmartDashboard.putNumber("MMTILP", mPidController.getP(SMART_MOTION_SLOT));
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

    private void gainSettings() {
        // PID coefficients
        kP = 5e-2;
        kI = 1e-6;
        kD = 0;
        kIz = 1;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        allowedErr = 1;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        // set PID coefficients
        mPidController.setP(kP);
        mPidController.setP(kP, SMART_MOTION_SLOT);
        mPidController.setI(kI);
        mPidController.setD(kD);
        mPidController.setIZone(kIz);
        mPidController.setFF(kFF);
        mPidController.setOutputRange(kMinOutput, kMaxOutput);
        mPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

}
