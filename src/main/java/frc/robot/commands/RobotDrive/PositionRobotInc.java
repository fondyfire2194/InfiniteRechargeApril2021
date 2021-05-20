// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class PositionRobotInc extends CommandBase {
  /** Creates a new PositionRobot. */
  private final RevDrivetrain m_drive;
  private double m_position;
  private double m_startTime;

  public PositionRobotInc(RevDrivetrain m_robotDrive, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = m_robotDrive;
    m_position = position;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_drive.leftTargetPosition += m_position;
    m_drive.rightTargetPosition += m_position;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
     m_drive.driveDistance(m_position, m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + .25
        && (m_drive.getInPositionLeft() && Math.abs(m_drive.getLeftRate()) < .1)
        && (m_drive.getInPositionRight() && Math.abs(m_drive.getRightRate()) < .1);
  }
}