// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class PositionRobot extends CommandBase {
  /** Creates a new PositionRobot. */
  private final RevDrivetrain m_drive;
  private double m_position;
  private double m_startTime;
  private double m_startAngle;
  private double m_maxSpeed;

  public PositionRobot(RevDrivetrain m_robotDrive, double position, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = m_robotDrive;
    m_position = position;
    m_maxSpeed = maxSpeed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_drive.leftTargetPosition = m_position;
    m_drive.rightTargetPosition = m_position;
    m_startAngle = m_drive.getYaw();
    m_drive.setMaxVel(m_maxSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.positionDistance(m_position, m_position);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + .25 && m_drive.getInPosition() && m_drive.getStopped();

  }
}
