// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class StopRobot extends CommandBase {
  /** Creates a new StopRobot. */

  private RevDrivetrain m_robotDrive;

  public StopRobot(RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_robotDrive.getLeftRate()) < .2 && Math.abs(m_robotDrive.getRightRate()) < .2;
  }
}