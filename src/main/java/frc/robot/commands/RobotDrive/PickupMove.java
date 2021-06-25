// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class PickupMove extends CommandBase {
  /** Creates a new PickupMove. */
  private final RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_speed;
  private double currentSpeed;
  private double slowDownDistance = -1;

  public PickupMove(RevDrivetrain drive, double endpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_endpoint = endpoint;
    m_speed = speed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSpeed = m_speed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * endpoint is negative as is average distance subtracting makes ave dist
     * positive but remaining distance will be negative
     */
    double remainingDistance = m_endpoint - m_drive.getAverageDistance();

    if (remainingDistance > slowDownDistance)
      currentSpeed = m_speed * .85;

    m_drive.arcadeDrive(currentSpeed, -m_drive.getYaw() * .01);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_drive.getAverageDistance() < m_endpoint;

  }
}
