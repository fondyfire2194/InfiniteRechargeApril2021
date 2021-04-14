// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class AdjustTiltPositionTarget extends CommandBase {
  /** Creates a new AdjustPositionTarget. */
  private final RevTiltSubsystem m_tilt;
  private double m_position;
  private int loopCtr;

  public AdjustTiltPositionTarget(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    addRequirements(m_tilt);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCtr = 0;
    m_tilt.getEndpoint = true;
    m_position = m_tilt.targetAngle;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
    if (!m_tilt.getEndpoint && loopCtr > 2)
      m_tilt.targetAngle = m_position + m_tilt.endpoint;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_tilt.getEndpoint & loopCtr > 2 || loopCtr > 5;
  }
}
