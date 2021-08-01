package frc.robot.subsystems;

import java.util.Arrays;

//import com.kauailabs.navx.frc.AHRS;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.navx.NavxWrapper;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.DifferentialDrivetrainSimWrapper;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Sim2.CANEncoderSim;
import frc.robot.Sim2.SimSparkMax;
import frc.robot.Pref;
import frc.robot.SimpleCSVLogger;
import frc.robot.sim.BaseDrivetrainSubsystem;

public class RevDrivetrain extends BaseDrivetrainSubsystem {
    private static final DrivetrainConstants DRIVETRAIN_CONSTANTS = new NeoDrivetrainConstants();
    private final CANSparkMax rightLeader;
    private final CANSparkMax rightFollower;
    private final CANSparkMax leftLeader;
    private final CANSparkMax leftFollower;
    private final CANEncoder leftEncoder;
    private final CANEncoder rightEncoder;
 

    private final CANPIDController mLeftPidController;
    private final CANPIDController mRightPidController;

    private final AHRS mGyro;

    public Field2d fieldSim;

    private final DifferentialDrive mDrive;
    // private final DifferentialDriveOdometry mOdometry;
    private DifferentialDrivetrainSimWrapper mSimulator;
    private SimDouble simGyro;
    private CANEncoderSim simLeftEncoder;
    private CANEncoderSim simRightEncoder;
    private DifferentialDrivetrainSim simDrive;
    public double leftTargetPosition;
    public double rightTargetPosition;

    public double pset, iset, dset, ffset, izset;
    public double rpset, riset, rdset, rffset, rizset;

    public double startDistance;

    private int SMART_MOTION_SLOT = 0;

    private int VELOCITY_SLOT = 1;

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
    public boolean leftBurnOK;
    public boolean rightBurnOK;

    public SimpleCSVLogger driveLogger;

    public boolean driveLogInProgress;

    public boolean logDriveItems;

    public boolean endDriveFile;

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

    private int robotStoppedCtr;

    public boolean robotStoppedForOneSecond;


    public RevDrivetrain() {
        rightLeader = new SimSparkMax(CANConstants.DRIVETRAIN_RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftLeader = new SimSparkMax(CANConstants.DRIVETRAIN_LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftFollower = new SimSparkMax(CANConstants.DRIVETRAIN_LEFT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightFollower = new SimSparkMax(CANConstants.DRIVETRAIN_RIGHT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
    
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();
        rightLeader.setInverted(true);
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        setIdleMode(true);
        leftLeader.enableVoltageCompensation(12);
        leftFollower.enableVoltageCompensation(12);
        rightLeader.enableVoltageCompensation(12);
        rightFollower.enableVoltageCompensation(12);
    
        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();
    
        leftEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);
        rightEncoder.setVelocityConversionFactor(DriveConstants.METERS_PER_MOTOR_REV / 60.0);

        mLeftPidController = leftLeader.getPIDController();
        mRightPidController = rightLeader.getPIDController();

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftTargetPosition = 0;
        rightTargetPosition = 0;

        leftLeader.setInverted(false);
        rightLeader.setInverted(true);

        leftFollower.follow(leftLeader, false);
        rightFollower.follow(rightLeader, false);
        leftLeader.setOpenLoopRampRate(.5);
        rightLeader.setOpenLoopRampRate(.5);

        leftLeader.setClosedLoopRampRate(1);
        rightLeader.setClosedLoopRampRate(1);

        mGyro = new AHRS();

        mDrive = new DifferentialDrive(leftLeader, rightLeader);
        mDrive.setRightSideInverted(false);

        mDrive.setSafetyEnabled(false);

        driveLogger = new SimpleCSVLogger();

        tuneGains();

        setIdleMode(false);

        if (RobotBase.isSimulation()) {
            simLeftEncoder = new CANEncoderSim(false,CANConstants.DRIVETRAIN_LEFT_MASTER);
            simRightEncoder = new CANEncoderSim(false, CANConstants.DRIVETRAIN_RIGHT_MASTER);
            simGyro =
                new SimDouble(
                    SimDeviceDataJNI.getSimValueHandle(
                        SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));
            simDrive =
                new DifferentialDrivetrainSim(
                    LinearSystemId.identifyDrivetrainSystem(kV_lin, kA_lin, kV_ang, kA_ang),
                    DCMotor.getNEO(2),
                    GEARING,
                    TRACK_WIDTH,
                    WHEEL_RADIUS,
                    null);
          }
                 // the Field2d class lets us visualize our robot in the simulation GUI.
            fieldSim = new Field2d();
            // mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
            SmartDashboard.putData("Field", fieldSim);
        

        // Set current limiting on drive train to prevent brown outs
        // Arrays.asList(leftLeader, rightLeader, leftFollower, rightFollower)
        //         .forEach((SimableCANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        kP = .4;
        kI = .1;
        kD = .5;
        maxAcc = 8;
        maxVel = 3;

    
    }

    /////////////////////////////////////
    // Accessors
    /////////////////////////////////////
    @Override
    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    @Override
    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    public double getAverageDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    @Override
    public double getLeftRate() {
        return leftEncoder.getVelocity();
    }

    @Override
    public double getRightRate() {
        return rightEncoder.getVelocity();
    }

    public double getRightOut() {
        return rightLeader.getAppliedOutput();
    }

    public double getRightAmps() {
        return rightLeader.getOutputCurrent();
    }

    public double getLeftOut() {
        return leftLeader.getAppliedOutput();
    }

    public double getLeftAmps() {
        return leftLeader.getOutputCurrent();
    }

    public double getRightFollowerOut() {
        return rightFollower.getAppliedOutput();
    }

    public double getLeftFollowerOut() {
        return leftFollower.getAppliedOutput();
    }

    public boolean getLeftFollower() {
        return leftFollower.isFollower();
    }

    public boolean getRightFollower() {
        return rightFollower.isFollower();
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
        leftLeader.setVoltage(left);
        rightLeader.setVoltage(right);
        mDrive.feed();
    }

    @Override
    public void smartVelocityControlMetersPerSec(double leftVelocityMetersPerSec, double rightVelocityMetersPerSec) {
        mLeftPidController.setReference(leftVelocityMetersPerSec, ControlType.kVelocity, VELOCITY_SLOT);
        mRightPidController.setReference(rightVelocityMetersPerSec, ControlType.kVelocity, VELOCITY_SLOT);
        mDrive.feed();
    }

    @Override
    public void driveDistance(double leftPosition, double rightPosition) {
        mLeftPidController.setReference(leftPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mDrive.feed();
    }

    public void resetPID() {
        mLeftPidController.setIAccum(0);
        mRightPidController.setIAccum(0);
    }

    public void positionDistance(double leftPosition, double rightPosition) {

        mLeftPidController.setReference(leftPosition, ControlType.kPosition, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kPosition, SMART_MOTION_SLOT);
        mDrive.feed();
    }

    @Override
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        // resetSimOdometry(getPose());
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

        if (!isStopped()) {
            robotStoppedCtr = 0;
        }

        if (isStopped() && robotStoppedCtr <= 50)
            robotStoppedCtr++;

        robotStoppedForOneSecond = isStopped() && robotStoppedCtr >= 50;

    }

    public boolean checkCAN() {
        leftLeadConnected = leftLeader.getFirmwareVersion() != 0;
        rightLeadConnected = rightLeader.getFirmwareVersion() != 0;
        leftFollowerConnected = leftFollower.getFirmwareVersion() != 0;
        rightFollowerConnected = rightFollower.getFirmwareVersion() != 0;
        allConnected = leftLeadConnected && leftFollowerConnected && rightLeadConnected && rightFollowerConnected;

        return allConnected;
    }

    public void resetAll() {
        resetGyro();
        resetEncoders();
    }

    @Override
    public void simulationPeriodic() {
      var volts = RobotController.getInputVoltage();
      simDrive.setInputs(volts * leftLeader.get(), volts * rightLeader.get());
  
      simDrive.update(0.02);
  
      simLeftEncoder.setPosition(simDrive.getLeftPositionMeters());
      simLeftEncoder.setVelocity(simDrive.getLeftVelocityMetersPerSecond());
      simRightEncoder.setPosition(simDrive.getRightPositionMeters());
      simRightEncoder.setVelocity(simDrive.getRightPositionMeters());
      simGyro.set(-simDrive.getHeading().getDegrees());
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(simDrive.getCurrentDrawAmps()));
    }

    @Override
    protected void resetSimOdometry(Pose2d pose) {
        mSimulator.resetOdometry(pose);
    }
    // /**
    // * Updates the field-relative position.
    // */
    // public void updateOdometry() {
    // var gyroAngle = getYaw();
    // mOdometry.update(gyroAngle, getLeftDistance(), getRightDistance());

    // }

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
        return Math.abs(leftEncoder.getVelocity()) < .2;
    }

    public boolean gyroStopped() {
        return !mGyro.isMoving();
    }

    public boolean gyroRotating() {
        return mGyro.isRotating();
    }

    public double getHeading() {
        return Math.IEEEremainder(mGyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetPose(Pose2d pose) {
        // The left and right encoders MUST be reset when odometry is reset
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeadingDegrees()));
        if (RobotBase.isSimulation())
            resetSimOdometry(pose);
    }

    public void clearFaults() {
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.clearFaults());

    }

    public int getFaults() {
        return leftLeader.getFaults() + leftFollower.getFaults() + rightLeader.getFaults() + rightFollower.getFaults();
    }

    public boolean getInPositionLeft() {
        return Math.abs(leftTargetPosition - getLeftDistance()) < .25;
    }

    public boolean getInPositionRight() {
        return Math.abs(rightTargetPosition - getRightDistance()) < .25;
    }

    public boolean getInPosition() {
        return getInPositionLeft() && getInPositionRight();
    }

    public boolean getStopped() {
        return Math.abs(getLeftRate()) < .5 && Math.abs(getRightRate()) < .5;

    }

    /**
     * Attempts to follow the given drive states using offboard PID.
     *
     * @param left  The left wheel state.
     * @param right The right wheel state.
     */
    public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {

        mLeftPidController.setReference(left.position, ControlType.kPosition);

        m_feedforward.calculate(left.velocity);

        mRightPidController.setReference(right.position, ControlType.kPosition);

        m_feedforward.calculate(right.velocity);
    }

    public void setIdleMode(boolean brake) {
        if (brake) {

            // Set motors to brake when idle. We don't want the drive train to coast.
            Arrays.asList(leftLeader, rightLeader, leftFollower, rightFollower)
                    .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
        }

        else {
            // Set motors to brake when idle. We don't want the drive train to coast.
            Arrays.asList(leftLeader, rightLeader, leftFollower, rightFollower)
                    .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));

        }
    }

    public void setMaxVel(double maxVel) {

        mLeftPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
        mRightPidController.setSmartMotionMaxVelocity(maxVel, SMART_MOTION_SLOT);
    }

    public double getMatchTime() {
        return DriverStation.getInstance().getMatchTime();
    }

    private void setVGains() {
        mLeftPidController.setFF(kFF, VELOCITY_SLOT);
        mRightPidController.setFF(kFF, VELOCITY_SLOT);

        mLeftPidController.setP(kP, VELOCITY_SLOT);
        mRightPidController.setP(kP, VELOCITY_SLOT);

        mLeftPidController.setI(kI, VELOCITY_SLOT);
        mRightPidController.setI(kI, VELOCITY_SLOT);

        mLeftPidController.setD(kD, VELOCITY_SLOT);
        mRightPidController.setD(kD, VELOCITY_SLOT);

    }

    @Override
    public void close() {
        leftLeader.close();
        leftFollower.close();
        rightLeader.close();
        rightFollower.close();
    }

    private void tuneGains() {

        kP = Pref.getPref("dRKp");
        kI = Pref.getPref("dRKi");
        kD = Pref.getPref("dRKd");

        kIz = Pref.getPref("dRKiz");
        kFF = Pref.getPref("dRKff");// 90 rps * .0467 = 4.2 meters per second. 1/4.2 = .238 kff

        setVGains();
        getGains();

    }

    private void getGains() {
        ffset = mLeftPidController.getFF(VELOCITY_SLOT);
        pset = mLeftPidController.getP(VELOCITY_SLOT);
        rffset = mRightPidController.getFF(VELOCITY_SLOT);
        rpset = mRightPidController.getP(VELOCITY_SLOT);

    }

    private void checkTune() {

        tuneOn = Pref.getPref("dRTune") == 1. && allConnected;

        if (tuneOn && !lastTuneOn) {

            tuneGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;
    }

}
