/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.RevDrivetrain;

/**
 * Add your docs here.
 */
public class FondyFireTrajectory {

        public Trajectory crossLine, example;
        public Trajectory trenchStartOne, trenchStartTwo, trenchStartThree;
        public Trajectory controlPanelStartOne, controlPanelStartTwo;
        public Trajectory centerStart, leftStart, rightStart;
        public Trajectory leftStartCurve, rightStartCurve;
        private RevDrivetrain m_drive;
        private static double bottomRectAngle = -Math.toRadians(67.5);
        private static double topRectAngle = -Math.toRadians(22.5);

        public FondyFireTrajectory(RevDrivetrain drive) {
                m_drive = drive;

                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, 11); // 8

                TrajectoryConfig configReversed = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
                configReversed.setReversed(true);

                TrajectoryConfig configPickupReversed = new TrajectoryConfig(DriveConstants.kPickupSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);
                configReversed.setReversed(true);

                TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.kMaxTrajectoryMetersPerSecond,
                                DriveConstants.kMaxTrajectoryAccelerationMetersPerSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                // // straight line
                // crossLine = TrajectoryGenerator.generateTrajectory(
                //                 List.of(new Pose2d(13, 0, new Rotation2d(0)), new Pose2d(12, 0, new Rotation2d(0))),
                //                 configReversed);

                // straight line opposite power port
                centerStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, 5.7, new Rotation2d(0)),
                                new Pose2d(11, 4.5, new Rotation2d(0))), configReversed);

                // straight line left of power port
                leftStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, 5.2, new Rotation2d(0)),
                                new Pose2d(12, 5.2, new Rotation2d(0))), configReversed);

                // straight line right of power port
                rightStart = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, 6.3, new Rotation2d(0)),
                                new Pose2d(12, 6.3, new Rotation2d(0))), configReversed);

                // start left of power port curve back for straight shot
                leftStartCurve = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(13, 5.2, new Rotation2d(0)),
                                new Pose2d(12, 5.3, new Rotation2d(0))), configReversed);

                // start right of power port curve back for straight shot
                rightStartCurve = TrajectoryGenerator
                                .generateTrajectory(
                                                List.of(new Pose2d(13, 6.3, new Rotation2d(0)),
                                                                new Pose2d(12, 5.3, new Rotation2d(0))),
                                                configReversed);

                // zig zag
                example = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(2, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(3, 1), new Translation2d(4, 1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(2, 0, new Rotation2d(0)),
                                // Pass config
                                configForward);

                trenchStartOne = TrajectoryGenerator.generateTrajectory(
                                
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
                                new Pose2d(4, 0.1, new Rotation2d(0)),
                                // pass config
                                configPickupReversed);

                trenchStartTwo = TrajectoryGenerator.generateTrajectory(

                                // List.of(new Pose2d(6.5, 0, new Rotation2d(0)), new Pose2d(2, 0, new
                                // Rotation2d(0))),
                                new Pose2d(6.5, 0, new Rotation2d(0)),
                                List.of(new Translation2d(4.5, 0), new Translation2d(3, 0)),
                                new Pose2d(2.0, 0.1, new Rotation2d(0)),
                                // pass config
                                configPickupReversed);

                trenchStartThree = TrajectoryGenerator.generateTrajectory(
                                // List.of(new Pose2d(4, 0, new Rotation2d(0)), new Pose2d(1, 0, new
                                // Rotation2d(0))),
                                new Pose2d(4, 0, new Rotation2d(0)),
                                List.of(new Translation2d(3, 0), new Translation2d(2, 0)),
                                new Pose2d(1, 0.1, new Rotation2d(0)),
                                // pass config
                                configForward);

                // controlPanelStartOne = TrajectoryGenerator.generateTrajectory(
                // List.of(new Pose2d(13, .71, new Rotation2d(0)), new Pose2d(12, .71, new
                // Rotation2d(0))),
                // configPickupReversed);

                // controlPanelStartTwo = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // List.of(new Pose2d(12, .71, new Rotation2d(0)), new Pose2d(12, 6, new
                // Rotation2d(.2))),
                // // Pass config
                // configForward);

        }

        public RamseteCommand getRamsete(Trajectory traj) {
                return new RamseteCommand(traj, m_drive::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
        }
}
