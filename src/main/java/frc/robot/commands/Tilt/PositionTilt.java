// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private int loopCtr;

  private double m_endpoint;

  public PositionTilt(RevTiltSubsystem tilt, double endpoint) {
    m_tilt = tilt;
    m_endpoint = endpoint;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    loopCtr = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
    m_tilt.targetAngle = m_endpoint;
    m_tilt.goToPosition(m_tilt.targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tilt.atTargetAngle()&&loopCtr>10;
  }
}
