// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class RobotPositionPlus extends CommandBase {
  /** Creates a new RobotPosition. */

  private boolean myEndItNow;

  private double rampIncrement;

  private boolean doneAccelerating;
  public static double currentMaxSpeed;
  private double remainingDistance = 0.0;
  private double startingTargetAngle;
  private boolean motionStarted;
  private boolean finalEndpointSet;

  private double slowDownDistance = 1;
  private double minimumSpeed = .25;
  private final RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_speed;
  private boolean decelerating;

  public RobotPositionPlus(RevDrivetrain drive, double speed, double endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_endpoint = endpoint;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rampIncrement = m_speed / 50;

    motionStarted = false;
    currentMaxSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!doneAccelerating) {
      currentMaxSpeed = currentMaxSpeed + rampIncrement;
      if (currentMaxSpeed > m_speed) {
        currentMaxSpeed = m_speed;
        doneAccelerating = true;
      }
    }
    if (decelerating) {
      currentMaxSpeed = m_speed * remainingDistance / slowDownDistance;

      if (currentMaxSpeed < minimumSpeed) {
        currentMaxSpeed = minimumSpeed;
      }
    }
    m_drive.arcadeDrive(currentMaxSpeed, -m_drive.getYaw() * .01);

    remainingDistance = m_endpoint - m_drive.getAverageDistance();
    if (remainingDistance < slowDownDistance) {
      decelerating = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
