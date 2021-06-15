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
import frc.robot.Pref;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.RevDrivetrain;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profile.
 */
public class PositionOneSideProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  private double m_startAngle = 0;

  public PositionOneSideProfiled(RevDrivetrain drive, double targetDistanceMeters) {

    super(

        new ProfiledPIDController(DriveConstants.kPositionP, DriveConstants.kPositionI, DriveConstants.kPositionD,
            new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxPositionAccelerationMetersPerSSquared)),
        // Close loop on average distance
        drive::getLeftDistance,
        // Set reference to target
        targetDistanceMeters,
        // Pipe output to move robot
        (output, setpoint) -> drive.arcadeDrive(output, drive.getYaw() * Pref.getPref("dRStKp")),
        // Require the drive
        drive);

    // Set the controller to be continuous (because it is an angle controller)

    // Set the controller tolerance - the delta tolerance ensures the robot is
    // stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(.1);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}