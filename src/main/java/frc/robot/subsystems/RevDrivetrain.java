package frc.robot.subsystems;

import java.util.Arrays;

//import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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

    private final SimableCANSparkMax mLeadLeft; // NOPMD
    private final SimableCANSparkMax mFollowerLeft; // NOPMD

    private final SimableCANSparkMax mLeadRight; // NOPMD
    private final SimableCANSparkMax mFollowerRight; // NOPMD

    public final CANEncoder mRightEncoder;
    public final CANEncoder mLeftEncoder;

    private final CANPIDController mLeftPidController;
    private final CANPIDController mRightPidController;

    private final AHRS mGyro;

    private Field2d fieldSim;

    private final DifferentialDrive mDrive;

    private DifferentialDrivetrainSimWrapper mSimulator;

    public double leftTargetPosition;
    public double rightTargetPosition;
    private final NetworkTable m_customNetworkTable;

    private int m_robotPositionCtr;

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

        mLeftEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);
        mRightEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);

        mLeftEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);
        mRightEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);

        mLeftPidController = mLeadLeft.getPIDController();
        mRightPidController = mLeadRight.getPIDController();

        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        leftTargetPosition = 0;
        rightTargetPosition = 0;

        mLeadLeft.setInverted(false);
        mLeadRight.setInverted(false);

        mFollowerLeft.follow(mLeadLeft, false);
        mFollowerRight.follow(mLeadRight, false);

        mGyro = new AHRS();

        mDrive = new DifferentialDrive(mLeadLeft, mLeadRight);
        mDrive.setRightSideInverted(false);

        for (CANPIDController pidController : new CANPIDController[] { mLeftPidController, mRightPidController }) {
            setupPidController(pidController, .02, 0, 0, .00544, 144, 144);
        }
        mDrive.setSafetyEnabled(false);
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

        NetworkTable coordinateGuiContainer = NetworkTableInstance.getDefault().getTable("CoordinateGui");
        coordinateGuiContainer.getEntry(".type").setString("CoordinateGui");

        m_customNetworkTable = coordinateGuiContainer.getSubTable("RobotPosition");
        // Set current limiting on drive train to prevent brown outs
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

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

    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    @Override
    public double getLeftRate() {
        return mLeftEncoder.getVelocity();
    }

    @Override
    public double getRightRate() {
        return mRightEncoder.getVelocity();
    }

    public double getRightOut() {
        return mLeadRight.getAppliedOutput();
    }

    public double getRightAmps() {
        return mLeadRight.getOutputCurrent();
    }

    public double getLeftOut() {
        return mLeadLeft.getAppliedOutput();
    }

    public double getLeftAmps() {
        return mLeadLeft.getOutputCurrent();
    }

    public double getRightFollowerOut() {
        return mFollowerRight.getAppliedOutput();
    }

    public double getLeftFollowerOut() {
        return mFollowerLeft.getAppliedOutput();
    }

    @Override
    public double getHeadingDegrees() {
        return mGyro.getAngle();
    }

    public double getYaw() {
        return Math.IEEEremainder(mGyro.getAngle(), 360) * -1;
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

        if (Math.abs(speed) < .1)
            speed = 0;
        if (Math.abs(rotation) < .1)
            rotation = 0;
        SmartDashboard.putNumber("ASS", speed);
        SmartDashboard.putNumber("ASR", rotation);
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
        resetSimOdometry(getPose());
    }

    @Override
    protected void resetSimOdometry(Pose2d pose) {
        mSimulator.resetOdometry(pose);
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public double getX() {
        return getTranslation().getX();
    }

    public double getY() {
        return getTranslation().getY();
    }

    ///////////////////////////////
    // Life Cycle
    ///////////////////////////////
    @Override
    public void periodic() {
        updateOdometry();
        m_customNetworkTable.getEntry("X").setDouble(getX() * 39.37);
        m_customNetworkTable.getEntry("Y").setDouble(getY() * 39.37);
        m_customNetworkTable.getEntry("Angle").setDouble(getHeading());
        // Actually update the display every 5 loops = 100ms
        if (m_robotPositionCtr % 5 == 0) {
            m_customNetworkTable.getEntry("Ctr").setDouble(m_robotPositionCtr);

        }
        ++m_robotPositionCtr;
        
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
    }

    public void resetGyro() {
    }

    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetPose(Pose2d pose) {
    }

    public void clearFaults() {
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.clearFaults());

    }

    public int getFaults() {
        return mLeadLeft.getFaults() + mLeadRight.getFaults() + mFollowerLeft.getFaults() + mFollowerRight.getFaults();
    }
}
