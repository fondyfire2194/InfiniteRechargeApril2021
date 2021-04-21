/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static final double inchToMetersConversionFactor = 0.0254;
   public static final double MINIMUM_TURN_SPEED = 0;

   public static final class CANConstants {
      public static final int PDP = 1;

      public static final int DRIVETRAIN_LEFT_MASTER = 2;
      public static final int DRIVETRAIN_LEFT_FOLLOWER = 3;

      public static final int DRIVETRAIN_RIGHT_MASTER = 4;
      public static final int DRIVETRAIN_RIGHT_FOLLOWER = 5;

      public static final int LEFT_MOTOR = 6;
      public static final int RIGHT_MOTOR = 7;

      public static final int TURRET_ROTATE_MOTOR = 8;// turret

      public static final int TILT_MOTOR = 9;

      // talons

      public static final int REAR_MOTOR = 10;

      public static final int FRONT_ROLLER = 14;
      public static final int REAR_ROLLER = 12;
      public static final int LEFT_BELT_MOTOR = 13;
      public static final int RIGHT_BELT_MOTOR = 11;

      public static final int CLIMB_MOTOR = 15;
      public static final int CP_TURN_MOTOR = 16;
   }

   public static final class PDPConstants {

      // PDP slots
      public static final int DRIVETRAIN_LEFT_MOTOR_A_PDP = 1;
      public static final int DRIVETRAIN_LEFT_MOTOR_B_PDP = 2;
      public static final int DRIVETRAIN_RIGHT_MOTOR_A_PDP = 3;
      public static final int DRIVETRAIN_RIGHT_MOTOR_B_PDP = 4;
      public static final int ELEVATOR_MOTOR_A_PDP = 5;
      public static final int ELEVATOR_MOTOR_B_PDP = 6;
      public static final int SHOOTER_MOTOR_A_PDP = 7;
      public static final int SHOOTER_MOTOR_B_PDP = 8;

   }

   public static final class DriveConstants {

      /**
       * Neo brushless 4096 counts per rev Gearing 10 to 1 6" diameter wheels Neo
       * reports in revs so multiply rev by 4096 to get counts
       * 
       * 
       */
      // DIMENSIONS IN METERS

      public static double WHEEL_DIAMETER = .1524;// 6"
      public static double WHEEL_CIRCUMFERENCE = .4788;// meters
      public static double METERS_PER_MOTOR_REV = 0.0467;// pi * diameter /geaar ratio
      public static double NEO550_COUNTS_PER_REV = 4096;// not used
      public static double DRIVE_GEAR_RATIO = 10.25;

      public final static double WHEELBASE_WIDTH = .69;

      public static double ksVolts = .171;

      public static final double kPDriveVeSl = 1.15;

      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            WHEELBASE_WIDTH);
      public static final double kMaxSpeedMetersPerSecond = 8.0;
      public static final boolean kGyroReversed = false;
      /**
       * 
       * Robot turn constants
       */
      public static double kTurnP = .05;
      public static double kTurnI = 0.;
      public static double kTurnD = 0.;
      public static double kMaxTurnRateDegPerS = 10.;
      public static double kMaxTurnAccelerationDegPerSSquared = 5.;
      public static double kTurnRateToleranceDegPerS = 1.;
      public static double kTurnToleranceDeg = 2.;

      /**
       * 
       * Position constants
       */
      public static double kPositionRateToleranceMetersPerS = 0.1;
      public static double kPositionToleranceMeters = 0.1;
      public static double kPositionI = 0.;
      public static double kPositionP = .01;
      public static double kPositionD = .0;
      public static double kMaxPositionAccelerationMetersPerSSquared = 0.;

      /**
       * 
       * Trajectory constants
       */

      public static double kMaxTrajectoryMetersPerSecond = 2;
      public static double kMaxTrajectoryAccelerationMetersPerSquared = 5.;
      public static double kPDriveVel = 5;
      public static double kPickupSpeedMetersPerSecond = 1;

      /**
       * Simulation parameters FAKE!!
       * 
       */
      public static double kvVoltSecondsPerMeter = 2;
      public static double kaVoltSecondsSquaredPerMeter = .208;
      public static final double kvVoltSecondsPerRadian = 3.0;
      public static final double kaVoltSecondsSquaredPerRadian = 0.3;

      public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

      public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);

   }

   public static final class ControlPanelConstants {

   }

   public static final class ClimberConstants {

   }

   public static class HoodedShooterConstants {

      /**
       * Turret
       */
      public static final double TURRET_POSITION_RATE = 1;
      public static final double TILT_POSITION_RATE = 1;
      public static final double MAX_SPEED = 5500.;
      public static final double MIN_SPEED = 1500.;
      public static final double SPEED_INCREMENT = 250.;

      /**
       * 
       * Tilt
       */

      public static final double TILT_DEG_PER_ENCODER_REV = .00029;// 100:1 gear box added
      public static final double TILT_MIN_ANGLE = 59;
      public static final double TILT_MAX_ANGLE = 70;

      // turret

      public static final double TURRET_MAX_ANGLE = 100;
      public static final double TURRET_MIN_ANGLE = -100;
      public static final double TurretSpeed = 0.025;

      /**
       * 100 revs of turret motor turns an 18 tooth pinion one time There are 222
       * teeth in 360 degrees, so 1 tooth = 360/220 = 1.64 degrees So 18 teeth (100
       * revs) = 18 * 1.64 = 29.5 degrees and one motor rev is .295 degrees
       */
      public static final double TURRET_DEG_PER_MOTOR_REV = .295;

      
   }

   public static final class IntakeConstants {

      public static final double REAR_SPEED = .5;

   }

   public static final class CellTransportConstants {

      public static final double FRONT_PASS_SPEED = .5;
      public static final double FRONT_SHOOT_SPEED = .56;
      public static final double REAR_PASS_SPEED = -.5;
      public static final double REAR_SHOOT_SPEED = -.5;
      public static final double BELT_SPEED = -.5;

   }

   public static final class AutoConstants {

      public static final double kMaxAccelerationMetersPerSecondSquared = 10;

      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
   }

   public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;
      public static final int kSetupControllerPort = 3;
      public static final int kShootBoxControllerPort = 2;
   }

   public static final class FieldConstants {

      public static final double fieldWidth = Units.inchesToMeters(323);
      public static final double fieldLength = Units.inchesToMeters(626.25);
      public static final double initiationLine = Units.inchesToMeters(115);// meters from Alliance wall
      public static final double centerPowerPort = 2.4;// meters
      public static final double centerTrench = 0;
      public static final double trenchToSecondBall = 0;
      public static final double trenchToThirdBall = 0;

      public static final double trenchToUnderCP = 0;

      public static final double robotLength = 1.5;
      public static final double robotWidth = 0;


      public static final double powerPortTargetCenterHeight = Units.inchesToMeters(84);
      public static final double TARGET_HEIGHT = Units.inchesToMeters(94);
      public static final double BASE_CAMERA_HEIGHT = Units.inchesToMeters(26);
      public static final double MAX_CAMERA_HEIGHT = Units.inchesToMeters(27);
      public static final double CAMERA_RADIUS_OF_TURN = Units.inchesToMeters(8);
      public static final double CAMERA_BASE_ANGLE = 51.;
   }
}
