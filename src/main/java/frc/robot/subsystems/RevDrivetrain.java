package frc.robot.subsystems;

import java.security.spec.MGF1ParameterSpec;
import java.util.Arrays;

//import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Pref;
import frc.robot.sim.BaseDrivetrainSubsystem;

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

    public Field2d fieldSim;

    private final DifferentialDrive mDrive;

    private DifferentialDrivetrainSimWrapper mSimulator;

    public double leftTargetPosition;
    public double rightTargetPosition;

    public double startDistance;

    private int POSITION_SLOT = 0;
    private int SMART_MOTION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;

    public double lastkP, lastkp, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastMaxVel,
            lastMinVel, lastMaxAcc, lastAllowedErr;
    // start NetworkTables

    public boolean tuneOn = false;
    public boolean lastTuneOn;

    public boolean leftLeadConnected;
    public boolean rightLeadConnected;
    public boolean leftFollowerConnected;
    public boolean rightFollowerConnected;
    public boolean allConnected;

    public boolean lockedForVision;

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

        mLeadLeft.restoreFactoryDefaults();
        mFollowerLeft.restoreFactoryDefaults();
        mLeadRight.restoreFactoryDefaults();
        mFollowerRight.restoreFactoryDefaults();

        mRightEncoder = mLeadRight.getEncoder();
        mLeftEncoder = mLeadLeft.getEncoder();

        mLeftEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);
        mRightEncoder.setPositionConversionFactor(DriveConstants.METERS_PER_MOTOR_REV);

        mLeftEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);
        mRightEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);

        mLeftPidController = mLeadLeft.getPIDController();
        mRightPidController = mLeadRight.getPIDController();

        mLeftPidController.setOutputRange(-.5, .5, POSITION_SLOT);

        tuneGains();

        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        leftTargetPosition = 0;
        rightTargetPosition = 0;

        mLeadLeft.setInverted(false);
        mLeadRight.setInverted(true);

        mFollowerLeft.follow(mLeadLeft, false);
        mFollowerRight.follow(mLeadRight, false);

        mGyro = new AHRS();

        mDrive = new DifferentialDrive(mLeadLeft, mLeadRight);
        mDrive.setRightSideInverted(false);

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

        // Set current limiting on drive train to prevent brown outs
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

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

    public boolean getLeftFollower() {
        return mFollowerLeft.isFollower();
    }

    public boolean getRightFollower() {
        return mFollowerRight.isFollower();
    }

    @Override
    public DrivetrainConstants getConstants() {
        return DRIVETRAIN_CONSTANTS;
    }

    @Override
    public void arcadeDrive(double speed, double rotation) {

        if (Math.abs(speed) < .1 || lockedForVision)
            speed = 0;
        if (Math.abs(rotation) < .1)
            rotation = 0;
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
        mLeftPidController.setReference(leftPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mDrive.feed();
    }

    public void positionDistance(double leftPosition, double rightPosition) {

        mLeftPidController.setReference(leftPosition, ControlType.kPosition, POSITION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kPosition, POSITION_SLOT);
        mDrive.feed();
    }

    @Override
    public void resetEncoders() {
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        resetSimOdometry(getPose());
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
        checkTune();
    }

    public boolean checkCAN() {
        leftLeadConnected = mLeadLeft.getFirmwareVersion() != 0;
        rightLeadConnected = mLeadRight.getFirmwareVersion() != 0;
        leftFollowerConnected = mFollowerLeft.getFirmwareVersion() != 0;
        rightFollowerConnected = mFollowerRight.getFirmwareVersion() != 0;
        allConnected = leftLeadConnected && leftFollowerConnected && rightLeadConnected && rightFollowerConnected;

        return allConnected;
    }

    public void resetAll() {
        resetGyro();
        resetEncoders();
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
        fieldSim.setRobotPose(getPose());
    }

    @Override
    protected void resetSimOdometry(Pose2d pose) {
        mSimulator.resetOdometry(pose);
    }

    @Override
    public double getHeadingDegrees() {
        return mGyro.getAngle();
    }

    public double getYaw() {
        return mGyro.getYaw();
        // return Math.IEEEremainder(mGyro.getAngle(), 360) * -1;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public void resetGyro() {
        mGyro.reset();
    }

    public boolean isStopped() {
        return Math.abs(mLeftEncoder.getVelocity()) < .2;
    }

    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetPose(Pose2d pose) {
        // The left and right encoders MUST be reset when odometry is reset
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
        mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeadingDegrees()));
        if (RobotBase.isSimulation())
            resetSimOdometry(pose);
    }

    // public void setRobotFromFieldPose() {
    // // only applies for simulation
    // if (RobotBase.isSimulation())
    // setPose(fieldSim.getRobotPose());
    // }

    public void clearFaults() {
        Arrays.asList(mLeadLeft, mLeadRight, mFollowerLeft, mFollowerRight)
                .forEach((SimableCANSparkMax spark) -> spark.clearFaults());

    }

    public int getFaults() {
        return mLeadLeft.getFaults() + mLeadRight.getFaults() + mFollowerLeft.getFaults() + mFollowerRight.getFaults();
    }

    public boolean getInPositionLeft() {
        return Math.abs(leftTargetPosition - getLeftDistance()) < allowedErr;
    }

    public boolean getInPositionRight() {
        return Math.abs(rightTargetPosition - getRightDistance()) < allowedErr;
    }

    public void setMaxVel(double maxVel) {
        mLeftPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
        mRightPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

    public void calibratePID(final double p, final double i, final double d, double kIz, double acc, double kFF,
            int slotNumber) {
        if (p != lastkP) {
            mLeftPidController.setP(p, slotNumber);
            mRightPidController.setP(p, slotNumber);
            lastkP = p;

        }

        if (i != lastkI) {
            mLeftPidController.setI(i, slotNumber);
            mRightPidController.setI(i, slotNumber);
            lastkI = i;
        }

        if (d != lastkD) {
            mLeftPidController.setI(d, slotNumber);
            mRightPidController.setD(d, slotNumber);
            lastkD = d;

            if (kIz != lastkIz) {
                mLeftPidController.setIZone(kIz, slotNumber);
                mRightPidController.setIZone(kIz, slotNumber);
                lastkIz = kIz;
            }

            mLeftPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
            mLeftPidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
            mLeftPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
            mLeftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);
            mLeftPidController.setFF(kFF, SMART_MOTION_SLOT);
            mLeftPidController.setOutputRange(kMinOutput, kMaxOutput);
            mRightPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
            mRightPidController.setSmartMotionMinOutputVelocity(minVel, SMART_MOTION_SLOT);
            mRightPidController.setSmartMotionMaxAccel(maxAcc, SMART_MOTION_SLOT);
            mRightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, SMART_MOTION_SLOT);
            mRightPidController.setFF(kFF, SMART_MOTION_SLOT);
            mRightPidController.setOutputRange(kMinOutput, kMaxOutput);
        }

    }

    private void setGains() {

        fixedSettings();

        kP = .1;

        kI = 0;
        kD = .0;
        kIz = 0;

        // set PID coefficients

        calibratePID(kP, kI, kD, kIz, maxAcc, kFF, SMART_MOTION_SLOT);

    }

    private void tuneGains() {

        fixedSettings();

        double kp = Pref.getPref("dRKp");
        double i = Pref.getPref("dRKi");
        double d = Pref.getPref("dRKd");
        double iz = Pref.getPref("dRKiz");

        calibratePID(kp, i, d, iz, maxAcc, kFF, SMART_MOTION_SLOT);

    }

    private void fixedSettings() {
        kFF = .0038;// 1/5,700 rpm = 1.75e-4 1 rev = .0467 meters so max mperm = 5700 * .0467 = 266
                    // and 1/266 = .0038
        kMaxOutput = .75;// 266 mpm = 4 mps limiting to 3mps
        kMinOutput = -.75;

        maxVel = 180;// mpm 3/sec
        maxAcc = 10;// mpmpsec
        allowedErr = .1;

    }

    private void checkTune() {
        tuneOn = Pref.getPref("dRTune") != 0.;

        if (tuneOn && !lastTuneOn) {

            tuneGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;
    }

}
