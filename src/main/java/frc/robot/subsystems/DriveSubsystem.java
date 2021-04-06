/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.SimpleCSVLogger;
import frc.robot.simulation.SparkMaxWrapper;
import frc.robot.FieldMap;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
   public CANSparkMax m_leftMaster = new SparkMaxWrapper(CANConstants.DRIVETRAIN_LEFT_MASTER, MotorType.kBrushless);
   public CANSparkMax m_leftFollower = new SparkMaxWrapper(CANConstants.DRIVETRAIN_LEFT_FOLLOWER, MotorType.kBrushless);
   public CANSparkMax m_rightMaster = new SparkMaxWrapper(CANConstants.DRIVETRAIN_RIGHT_MASTER, MotorType.kBrushless);
   public CANSparkMax m_rightFollower = new SparkMaxWrapper(CANConstants.DRIVETRAIN_RIGHT_FOLLOWER,
         MotorType.kBrushless);

   private final CANEncoder m_leftMaster_encoder = m_leftMaster.getEncoder();
   private final CANEncoder m_leftFollower_encoder = m_leftFollower.getEncoder();
   private final CANEncoder m_rightMaster_encoder = m_rightMaster.getEncoder();
   private final CANEncoder m_rightFollower_encoder = m_rightFollower.getEncoder();

   private Encoder leftEncoder = new Encoder(3, 4);
   private Encoder rightEncoder = new Encoder(5, 6);
   private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftMaster, m_leftFollower);

   private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightMaster, m_rightFollower);

   private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

   private final DifferentialDriveOdometry m_odometry;

   public static AHRS imu;

   private DifferentialDrivetrainSim drivetrainSimulator;
   private EncoderSim leftEncoderSim;
   private EncoderSim rightEncoderSim;
   // The Field2d class simulates the field in the sim GUI. Note that we can have
   // only one instance!
   private Field2d fieldSim;
   private SimDouble gyroAngleSim;

   private int displaySelect;

   public double[] distanceArray2x = new double[50];


   private final PIDController m_leftPIDController = new PIDController(1.45, 0, 0);
   private final PIDController m_rightPIDController = new PIDController(1.45, 0, 0);

   public SimpleCSVLogger simpleCSVLogger;
   public boolean endFile;
   public boolean endTrajFile;
   public List<Integer> sparkMaxID;
   public List<CANSparkMax> motor;
   private int prevBallLocation = 0;
   private int prevStartLocation = 10;

   // Put methods for controlling this subsystem
   // here. Call these from Commands.
   public DriveSubsystem() {

      m_leftMaster.restoreFactoryDefaults();

      m_leftFollower.restoreFactoryDefaults();

      m_rightMaster.restoreFactoryDefaults();

      m_rightFollower.restoreFactoryDefaults();

      leftEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
      rightEncoder.setDistancePerPulse(DriveConstants.DISTANCE_PER_PULSE);
      m_drive.setSafetyEnabled(false);
      setLeftBrakeOn(false);
      setRightBrakeOn(false);
      simpleCSVLogger = new SimpleCSVLogger();
      if (Robot.isReal()) {
         try {
            // imu = new AHRS(I2C.Port.kOnboard);

            // imu = new AHRS(SPI.Port.kMXP);
            imu = new AHRS(SerialPort.Port.kUSB1);

         } catch (Exception ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
         }

         SmartDashboard.putString("NavX FW#", imu.toString());
      } else
         imu = new AHRS(Port.kMXP, (byte) 200);
      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
      // m_rangeSensor.setRangingMode(RangingMode.Short, 40);
      if (RobotBase.isSimulation()) {
         // If our robot is simulated
         // This class simulates our drivetrain's motion around the field.
         drivetrainSimulator = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
               DriveConstants.kDriveGearbox, DriveConstants.DRIVE_GEAR_RATIO, DriveConstants.WHEELBASE_WIDTH,
               DriveConstants.WHEEL_DIAMETER / 2.0, null);
         // The encoder and gyro angle sims let us set simulated sensor readings
         leftEncoderSim = new EncoderSim(leftEncoder);
         rightEncoderSim = new EncoderSim(rightEncoder);

         // get the angle simulation variable
         // SimDevice is found by name and index, like "name[index]"
         gyroAngleSim = new SimDeviceSim("navX-Sensor[0]").getDouble("Yaw");

         // the Field2d class lets us visualize our robot in the simulation GUI.
         fieldSim = new Field2d();
         SmartDashboard.putData("Field", fieldSim);
      }

   }

   @Override
   public void periodic() {

      // This method will be called once per scheduler run

      updateOdometry();
      SmartDashboard.putBoolean("RBIR", Robot.isReal());
      SmartDashboard.putBoolean("RBIS", Robot.isSimulation());
      displaySelect++;
      if (displaySelect >= 25) {
         displaySelect = 0;

      }
   }

   /*
    * Drives the robot using arcade controls
    *
    * @param fwd the commanded forward movement
    * 
    * @param rot the commanded rotation
    */
   public void arcadeDrive(double fwd, double rot) {
      if (Math.abs(fwd) < .1)
         fwd = 0;
      if (Math.abs(rot) < .1)
         rot = 0;
      SmartDashboard.putNumber("ARCF", fwd);
      m_drive.arcadeDrive(fwd, rot);
   }

   public void tankDrive(double left, double right) {
      m_drive.tankDrive(left, right);
   }

   /**
    * Drives the robot with the given linear velocity and angular velocity.
    *
    * @param xSpeed Linear velocity in m/s.
    * @param rot    Angular velocity in rad/s.
    */
   @SuppressWarnings("ParameterName")
   public void drive(double xSpeed, double rot) {
      var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
      setSpeeds(wheelSpeeds);
   }

   /**
    * Sets the desired wheel speeds.
    *
    * @param speeds The desired wheel speeds.
    */
   public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
      double leftOutput = m_leftPIDController.calculate(getLeftEncoderMps(), speeds.leftMetersPerSecond);
      double rightOutput = m_rightPIDController.calculate(getRightEncoderMps(), speeds.rightMetersPerSecond);
      m_leftMaster.set(leftOutput);
      m_rightMaster.set(rightOutput);
   }

   /**
    * Returns the angle of the robot as a Rotation2d.
    *
    * @return The angle of the robot.
    */
   public double getHeading() {
      return Math.IEEEremainder(imu.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

   }

   /**
    * Returns the angle of the robot as a Rotation2d.
    *
    * @return The angle of the robot.
    */
   public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(getHeading());
   }

   /**
    * Updates the field-relative position.
    */
   public void updateOdometry() {
      var gyroAngle = getAngle();
      m_odometry.update(gyroAngle, getLeftDistanceMeters(), getRightDistanceMeters());

   }

   public double getGyroYaw() {
      return imu.getYaw();
   }

   public double getGyroAngle() {
      return -imu.getAngle();

   }

   public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
   }

   public void resetGyro() {
      imu.reset();
   }

   public void setPose(Pose2d pose) {
      if (RobotBase.isSimulation()) {
         // This is a bit hokey, but if the Robot jumps on the field, we need
         // to reset the internal state of the DriveTrainSimulator.
         // No method to do it, but we can reset the state variables.
         // NOTE: this assumes the robot is not moving, since we are not resetting
         // the rate variables.
         drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

         // reset the GyroSim to match the driveTrainSim
         // do it early so that "real" odometry matches this value
         gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
         fieldSim.setRobotPose(pose);
      }

      // The left and right encoders MUST be reset when odometry is reset
      leftEncoder.reset();
      rightEncoder.reset();
      m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
   }

   public void resetPose(Pose2d pose) {

      resetEncoders();
      resetPose(pose);
   }

   public void resetAll() {
      resetGyro();
      resetEncoders();
   }

   public Pose2d getPose() {
      return m_odometry.getPoseMeters();
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

   public double getLeftDistanceMeters() {
      if (Robot.isReal()) {
         return m_leftMaster_encoder.getPosition() * DriveConstants.METERS_PER_MOTOR_REV;
      } else {
         return leftEncoder.getDistance();
      }
   }

   public double getRightDistanceMeters() {
      if (Robot.isReal()) {
         return m_rightMaster_encoder.getPosition() * DriveConstants.METERS_PER_MOTOR_REV;
      } else {
         return rightEncoder.getDistance();
      }
   }

   public double getAverageDistanceMeters() {
      return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2;
   }

   public double getLeftEncoderMps() {
      if (Robot.isReal()) {
         return m_leftMaster_encoder.getVelocity() * DriveConstants.METERS_PER_MOTOR_REV / 60;
      } else {
         return leftEncoder.getRate();
      }
   }

   public double getRightEncoderMps() {
      if (Robot.isReal()) {
         return m_rightMaster_encoder.getVelocity() * DriveConstants.METERS_PER_MOTOR_REV / 60;
      } else {
         return rightEncoder.getRate();
      }
   }

   public void resetEncoders() {
      if (Robot.isReal()) {
         m_leftMaster_encoder.setPosition(0);
         m_rightMaster_encoder.setPosition(0);
      }
   }

   private void setLeftBrakeOn(boolean on) {
      if (on) {
         m_leftMaster.setIdleMode(IdleMode.kBrake);
         m_leftFollower.setIdleMode(IdleMode.kBrake);
      } else {
         m_leftMaster.setIdleMode(IdleMode.kCoast);
         m_leftFollower.setIdleMode(IdleMode.kCoast);
      }
   }

   private void setRightBrakeOn(boolean on) {
      if (on) {
         m_rightMaster.setIdleMode(IdleMode.kBrake);
         m_rightFollower.setIdleMode(IdleMode.kBrake);
      } else {
         m_rightMaster.setIdleMode(IdleMode.kCoast);
         m_rightFollower.setIdleMode(IdleMode.kCoast);
      }
   }

   /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
   public void tankDriveVolts(double leftVolts, double rightVolts) {
      if (!Robot.isSimulation()) {
         m_leftMaster.setVoltage(leftVolts);
         m_rightMaster.setVoltage(-rightVolts);
         m_drive.feed();
      } else {
         m_leftMaster.set(leftVolts);
         m_rightMaster.set(-rightVolts);
         m_drive.feed();

      }
   }

   /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
   public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(getLeftEncoderMps(), getRightEncoderMps());
   }

   @Override
   public void simulationPeriodic() {
      // To update our simulation, we set motor voltage inputs, update the simulation,
      // and write the simulated positions and velocities to our simulated encoder and
      // gyro.
      // We negate the right side so that positive voltages make the right side
      // move forward.
      drivetrainSimulator.setInputs(m_leftMotors.get() * RobotController.getBatteryVoltage(),
            -m_rightMotors.get() * RobotController.getBatteryVoltage());
      drivetrainSimulator.update(0.020);
      SmartDashboard.putNumber("BAT", RobotController.getBatteryVoltage());
      SmartDashboard.putNumber("MLM", m_leftMotors.get());
      leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
      leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());

      rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
      rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());

      gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());

      fieldSim.setRobotPose(getPose());
   }

   public void moveAroundField() {
      // only applies for simulation
      if (RobotBase.isReal())
         return;

      int startPos = (int) SmartDashboard.getNumber("moveAroundField/startPos", 10);
      int ballPos = (int) SmartDashboard.getNumber("moveAroundField/ballPos", 0);

      // Use either start of ball to set robot pose.
      // 10 is the dummy default value for start.
      if (startPos != prevStartLocation && startPos >= 0 && startPos < FieldMap.startPosition.length) {
         // The start value has changed and is valid. Use it to position the robot.
         fieldSim.setRobotPose(FieldMap.startPosition[startPos]);
      } else if (ballPos != prevBallLocation && ballPos >= 0 && ballPos < FieldMap.ballPosition.length) {
         // start value is invalid, so use the ball position with 0 rotation angle
         fieldSim.setRobotPose(new Pose2d(FieldMap.ballPosition[ballPos], new Rotation2d(0.0)));
      }

      prevBallLocation = ballPos;
      prevStartLocation = startPos;

      // On every call, output the Pose info to SmartDashboard for debugging
      // convenience
      Pose2d pose = fieldSim.getRobotPose();
      SmartDashboard.putNumber("moveAroundField/robotX", Units.metersToInches(pose.getX()));
      SmartDashboard.putNumber("moveAroundField/robotY", Units.metersToInches(pose.getY()));
      SmartDashboard.putNumber("moveAroundField/robotAngle", pose.getRotation().getDegrees());
   }

   public void setRobotFromFieldPose() {
      // only applies for simulation
      if (RobotBase.isSimulation())
         setPose(fieldSim.getRobotPose());
   }

}
