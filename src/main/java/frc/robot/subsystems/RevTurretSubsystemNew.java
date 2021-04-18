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
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.sim.ShooterSubsystem;

public class RevTurretSubsystemNew extends SubsystemBase implements ShooterSubsystem {
    public static DCMotor kGearbox = DCMotor.getNeo550(1);
    private static final double DEG_PER_MOTOR_REV = HoodedShooterConstants.TURRET_DEG_PER_MOTOR_REV;
    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private ISimWrapper mSimulator;
    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
   // private final RevEncoderSimWrapper mRevEncoderSimWrapper;
    public double visionCorrection;
    public double targetAngle;
    private double inPositionBandwidth = 1;

    public RevTurretSubsystemNew() {
        
        m_motor = new SimableCANSparkMax(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);// 1 /
                                                                // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV);

        m_motor.setOpenLoopRampRate(5);


        gainSettings();
        if (RobotBase.isSimulation())
            mPidController.setP(0.006);

        mEncoder.setPosition(0);
        targetAngle = 0;

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("RPOS",mEncoder.getPosition());
    }

    @Override
    public void spinAtRpm(double rpm) {
        // requiredSpeed = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public double getRPM() {
        return mEncoder.getVelocity();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    public void moveManually(double speed) {
        m_motor.set(speed);
        targetAngle = getAngle();
    }


    public void goToPositionMotionMagic(double angle) {
        mPidController.setReference(angle, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    public void goToPosition(double angle) {
        mPidController.setReference(angle, ControlType.kPosition, SMART_MOTION_SLOT);
  

    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
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

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

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
        mSimulator.update();
        
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    private void gainSettings() {
        // PID coefficients
        kP = 5e-1;
        kI = 0;// 1e-5;
        kD = 0;
        kIz = 1;
        kFF = 0.00000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;// not used
        allowedErr = 1;
        // Smart Motion Coefficients
        maxVel = 10000; // rpm
        maxAcc = 750;

        // set PID coefficients

        mPidController.setP(kP, SMART_MOTION_SLOT);
        mPidController.setI(kI, SMART_MOTION_SLOT);
        mPidController.setD(kD, SMART_MOTION_SLOT);
        mPidController.setIZone(kIz, SMART_MOTION_SLOT);
        mPidController.setFF(kFF, SMART_MOTION_SLOT);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
        mPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

    public double getHeightInches() {
        // TODO Auto-generated method stub
        return 0;
    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }
}
