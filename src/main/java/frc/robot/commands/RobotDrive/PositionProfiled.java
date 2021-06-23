/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.RevDrivetrain;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profile.
 */
public class PositionProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */
  public PositionProfiled(RevDrivetrain drive, double targetMeters, double maxVel) {
    super(
        new ProfiledPIDController(drive.kP, drive.kI, drive.kD, new TrapezoidProfile.Constraints(maxVel, drive.maxAcc)),
        // Close loop on heading
        drive::getAverageDistance,
        // Set reference to target
        targetMeters,
        // Pipe output to move robot
        (output, setpoint) -> drive.arcadeDrive(output, -drive.getYaw() * .01),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(DriveConstants.kPositionToleranceMeters,
        DriveConstants.kPositionRateToleranceMetersPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}