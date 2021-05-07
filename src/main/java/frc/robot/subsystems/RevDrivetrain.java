package frc.robot.subsystems;

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

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;
    // start NetworkTables

    public boolean tuneOn = false;

    private int loopCtr;

    public boolean leftLeadConnected;
    public boolean rightLeadConnected;
    public boolean leftFollowerConnected;
    public boolean rightFollowerConnected;
    public boolean allConnected;

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

        if (!tuneOn)
            setGains();

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

        // for (CANPIDController pidController : new CANPIDController[] {
        // mLeftPidController, mRightPidController }) {
        // setupPidController(pidController, .2, 0, 0, .21, 3.5, 3);
        // }
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
        mLeftPidController.setReference(leftPosition, ControlType.kPosition, POSITION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kPosition, POSITION_SLOT);
        mDrive.feed();
    }

    public void positionDistance(double leftPosition, double rightPosition, double velocity) {

        mLeftPidController.setReference(leftPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        mRightPidController.setReference(rightPosition, ControlType.kSmartMotion, SMART_MOTION_SLOT);

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

        loopCtr++;
        if (loopCtr > 28) {
            tuneOn = Pref.getPref("dRTune") != 0.;

            if (tuneOn)
                tuneGains();

        }
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
        SmartDashboard.putString("Pose", getPose().toString());
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
        return Math.IEEEremainder(mGyro.getAngle(), 360) * -1;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public void resetGyro() {
        mGyro.reset();
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

    public boolean getInPosition() {
        return Math.abs(leftTargetPosition - getAverageDistance()) < .25;
    }

    public void calibrateLeftPID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        if (p != lastkP) {
            mLeftPidController.setP(p, slotNumber);
            lastkP = p;
        }
        if (i != lastkI) {
            mLeftPidController.setI(i, slotNumber);
            lastkI = i;
        }

        if (d != lastkD) {
            mLeftPidController.setD(d, slotNumber);
            lastkD = d;
        }
        if (f != lastkFF) {
            mLeftPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mLeftPidController.setIZone(kIz, slotNumber);
            lastkIz = kIz;
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mLeftPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mLeftPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mLeftPidController.setSmartMotionMaxVelocity(maxVel, slotNumber);
                lastmaxVel = maxVel;
            }

            if (allowedErr != lastallowedErr) {
                mLeftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);
                lastallowedErr = allowedErr;
            }

        }
    }

    public void calibrateRightPID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        if (p != lastkP) {
            mRightPidController.setP(p, slotNumber);
            lastkP = p;
        }
        if (i != lastkI) {
            mRightPidController.setI(i, slotNumber);
            lastkI = i;
        }

        if (d != lastkD) {
            mRightPidController.setD(d, slotNumber);
            lastkD = d;
        }
        if (f != lastkFF) {
            mRightPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mRightPidController.setIZone(kIz, slotNumber);
            lastkIz = kIz;
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mRightPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mRightPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mRightPidController.setSmartMotionMaxVelocity(maxVel, slotNumber);
                lastmaxVel = maxVel;
            }

            if (allowedErr != lastallowedErr) {
                mRightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);
                lastallowedErr = allowedErr;
            }
        }

    }

    private void setGains() {

        fixedSettings();
        kP = .000001;
        kI = 0;
        kD = .0005;
        kIz = 0;
        maxVel = 5000; // motor rev per min
        maxAcc = 7500;

        // set PID coefficients

        calibrateLeftPID(kP, kI, kD, kFF, kIz, POSITION_SLOT);
        calibrateRightPID(kP, kI, kD, kFF, kIz, POSITION_SLOT);
    }

    private void tuneGains() {

        fixedSettings();

        double p = Pref.getPref("dRKp");
        double i = Pref.getPref("dRKi");
        double d = Pref.getPref("dRKd");
        double iz = Pref.getPref("dRKiz");
        maxVel = Pref.getPref("dRMaxV");
        maxAcc = Pref.getPref("dRMaxA");

        calibrateLeftPID(p, i, d, kFF, iz, POSITION_SLOT);
        calibrateRightPID(p, i, d, kFF, iz, POSITION_SLOT);
    }

    private void fixedSettings() {
        kFF = .000078;//
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 11000;// not used
        allowedErr = 1;

    }
}
