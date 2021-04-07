package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import frc.robot.sim.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class RevTiltSubsystem extends SubsystemBase implements ElevatorSubsystem {
    private static final double GRAVITY_COMPENSATION_VOLTS = 0.;
    private static final double TICKS_PER_DEGREE = 4096;
    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    public DigitalInput m_reverseLimit = new DigitalInput(9);
    private ISimWrapper mElevatorSim;
    private int p;
    public double visionCorrection;
    public boolean positionResetDone;
    

    public RevTiltSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TILT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();

        mEncoder.setPositionConversionFactor(1 / HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV);

        mPidController.setP(0.16);
        m_motor.restoreFactoryDefaults();
        gainSettings();

        if (RobotBase.isSimulation()) {
            ElevatorSimConstants.kCarriageMass = 0;
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
        p++;
        SmartDashboard.putNumber("TiltPos", getHeightInches());
        // SmartDashboard.putNumber("P", p);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void moveManually(double speed) {
        m_motor.set(speed);
        SmartDashboard.putNumber("TEST", p);
    }

    @Override
    public void goToPosition(double degrees) {
        double motorRevs = degrees * HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV;

        mPidController.setReference(motorRevs, ControlType.kPosition, POSITION_SLOT, GRAVITY_COMPENSATION_VOLTS,
                CANPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void goToPositionMotionMagic(double inches) {
        double meters = Units.inchesToMeters(inches);

        mPidController.setReference(meters, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public void resetAngle(double position) {
        mEncoder.setPosition(position);
    }

    @Override
    public boolean isAtHeight(double inches, double allowableError) {
        return Math.abs(inches - getHeightInches()) < allowableError;
    }

    @Override
    public double getHeightInches() {
        return Units.metersToInches(mEncoder.getPosition());
    }

    public double getAngle() {
        return Units.metersToInches(mEncoder.getPosition());
    }

    public double getOut(){
        return m_motor.get();
    }

    public double getSpeed(){
        return mEncoder.getVelocity();
    }

    public boolean onPlusSoftwareLimit(){
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward);
     }
  
     public boolean onMinusSoftwareLimit(){
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
     }
    @Override
    public void simulationPeriodic() {
        mElevatorSim.update();
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    private void gainSettings() {
        // PID coefficients
        kP = 5e-3;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        // set PID coefficients
        mPidController.setP(kP);
        mPidController.setI(kI);
        mPidController.setD(kD);
        mPidController.setIZone(kIz);
        mPidController.setFF(kFF);
        mPidController.setOutputRange(kMinOutput, kMaxOutput);
        mPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

}
