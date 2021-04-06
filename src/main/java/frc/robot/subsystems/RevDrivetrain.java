package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.sim.BaseDrivetrainSubsystem;
import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

public class RevDrivetrain extends BaseDrivetrainSubsystem {
    private static final DrivetrainConstants DRIVETRAIN_CONSTANTS = new NeoDrivetrainConstants();
    private static final double ENCODER_CONSTANT = (1.0 / DriveConstants.DRIVE_GEAR_RATIO
            * DRIVETRAIN_CONSTANTS.getkWheelDiameterMeters() * Math.PI);

    private final SimableCANSparkMax mLeadLeft; // NOPMD
    private final SimableCANSparkMax mFollowerLeft; // NOPMD

    private final SimableCANSparkMax mLeadRight; // NOPMD
    private final SimableCANSparkMax mFollowerRight; // NOPMD

    private final CANEncoder mRightEncoder;
    private final CANEncoder mLeftEncoder;

    private final CANPIDController mLeftPidController;
    private final CANPIDController mRightPidController;

    private final AHRS mGyro;

    private Field2d fieldSim;

    private final DifferentialDrive mDrive;

    private DifferentialDrivetrainSimWrapper mSimulator;

    @Override
    public void close() {
        mLeadLeft.close();
        mFollowerLeft.close();
        mLeadRight.close();
        mFollowerRight.close();
    }

    public RevDrivetrain() {
        mLeadLeft = new SimableCANSparkMax(CANConstants.DRIVETRAIN_LEFT_MASTER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mFollowerLeft = new SimableCANSparkMax(CANConstants.DRIVETRAIN_LEFT_FOLLOWER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mLeadRight = new SimableCANSparkMax(CANConstants.DRIVETRAIN_RIGHT_MASTER,
                CANSparkMaxLowLevel.MotorType.kBrushless);
        mFollowerRight = new SimableCANSparkMax(CANConstants.DRIVETRAIN_RIGHT_FOLLOWER,
                CANSparkMaxLowLevel.MotorType.kBrushless);

        mRightEncoder = mLeadRight.getEncoder();
        mLeftEncoder = mLeadLeft.getEncoder();

        mLeftEncoder.setPositionConversionFactor(DriveConstants.DISTANCE_PER_PULSE);
        mRightEncoder.setPositionConversionFactor(DriveConstants.DISTANCE_PER_PULSE);

        mLeftEncoder.setVelocityConversionFactor(DriveConstants.DISTANCE_PER_PULSE / 60.0);
        mRightEncoder.setVelocityConversionFactor(DriveConstants.DISTANCE_PER_PULSE / 60.0);

        mLeftPidController = mLeadLeft.getPIDController();
        mRightPidController = mLeadRight.getPIDController();

        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);

        mLeadLeft.setInverted(false);
        mLeadRight.setInverted(true);

        mFollowerLeft.follow(mLeadLeft, false);
        mFollowerRight.follow(mLeadRight, false);

        mGyro = new AHRS();

        mDrive = new DifferentialDrive(mLeadLeft, mLeadRight);
        mDrive.setRightSideInverted(false);

        for (CANPIDController pidController : new CANPIDController[] { mLeftPidController, mRightPidController }) {
            setupPidController(pidController, .02, 0, 0, .00544, 144, 144);
        }

        if (RobotBase.isSimulation()) {
            mSimulator = new DifferentialDrivetrainSimWrapper(DRIVETRAIN_CONSTANTS.createSim(),
                    new RevMotorControllerSimWrapper(mLeadLeft), new RevMotorControllerSimWrapper(mLeadRight),
                    RevEncoderSimWrapper.create(mLeadLeft), RevEncoderSimWrapper.create(mLeadRight),
                    new NavxWrapper().getYawGyro());
            mSimulator.setRightInverted(false);
            mSimulator.setLeftPdpChannels(PDPConstants.DRIVETRAIN_LEFT_MOTOR_A_PDP,
                    PDPConstants.DRIVETRAIN_LEFT_MOTOR_B_PDP);
            mSimulator.setRightPdpChannels(PDPConstants.DRIVETRAIN_RIGHT_MOTOR_A_PDP,
                    PDPConstants.DRIVETRAIN_RIGHT_MOTOR_B_PDP);
            // the Field2d class lets us visualize our robot in the simulation GUI.
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);
        }
    }

    private void setupPidController(CANPIDController pidController, double kp, double ki, double kd, double kff,
            double maxVelocity, double maxAcceleration) {
        pidController.setP(Units.metersToInches(kp));
        pidController.setI(ki);
        pidController.setD(kd);
        pidController.setFF(Units.metersToInches(kff));
        pidController.setSmartMotionMaxVelocity(Units.inchesToMeters(maxVelocity), 0);
        pidController.setSmartMotionMaxAccel(Units.inchesToMeters(maxAcceleration), 0);
    }

    /////////////////////////////////////
    // Accessors
    /////////////////////////////////////
    @Override
    public double getLeftDistance() {
        return mLeftEncoder.getPosition();
    }

    @Override
    public double getRightDistance() {
        return mRightEncoder.getPosition();
    }

    @Override
    public double getLeftRate() {
        return mLeftEncoder.getVelocity();
    }

    @Override
    public double getRightRate() {
        return mRightEncoder.getVelocity();
    }

    @Override
    public double getHeadingDegrees() {
        return mGyro.getYaw();
    }

    @Override
    public DrivetrainConstants getConstants() {
        return DRIVETRAIN_CONSTANTS;
    }

    /////////////////////////////////////
    // Control
    /////////////////////////////////////
    @Override
    public void arcadeDrive(double speed, double rotation) {
        mDrive.arcadeDrive(speed, rotation);
    }

    @Override
    public void tankDriveVolts(double left, double right) {
        mLeadLeft.setVoltage(left);
        mLeadRight.setVoltage(right);
        mDrive.feed();
    }

    @Override
    public void smartVelocityControlMetersPerSec(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
        mLeftPidController.setReference(leftVelocityMetersPerSec, ControlType.kVelocity);
        mRightPidController.setReference(rightVelocityMetersPerSec, ControlType.kVelocity);
        mDrive.feed();
    }

    @Override
    public void driveDistance(double leftPosition, double rightPosition) {
        mLeftPidController.setReference(leftPosition, ControlType.kSmartMotion);
        mRightPidController.setReference(rightPosition, ControlType.kSmartMotion);
        mDrive.feed();
    }

    @Override
    public void resetEncoders() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    @Override
    protected void resetSimOdometry(Pose2d pose) {
        mSimulator.resetOdometry(pose);
    }

    ///////////////////////////////
    // Life Cycle
    ///////////////////////////////
    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
    }
}
