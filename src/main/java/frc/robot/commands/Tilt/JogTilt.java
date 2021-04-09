// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class JogTilt extends CommandBase {
  /** Creates a new JogTilt. */
  private final RevTiltSubsystem m_tilt;
  private double m_speed;

  public JogTilt(RevTiltSubsystem tilt, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_speed = speed;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tilt.moveManually(m_speed);
    SmartDashboard.putNumber("TLSP", m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.stop();
    m_tilt.targetAngle=m_tilt.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
