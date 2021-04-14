// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class StopTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private double m_position;

  public StopTilt(RevTiltSubsystem tilt) {
    m_tilt = tilt;
    m_position = m_tilt.getAngle();
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.targetAngle = m_position;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tilt.goToPositionMotionMagic(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_tilt.targetAngle = m_tilt.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}