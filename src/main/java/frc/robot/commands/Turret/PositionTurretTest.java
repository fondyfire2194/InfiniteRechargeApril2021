// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretTest extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private int loopCtr;

  public PositionTurretTest(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;

    addRequirements(m_turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCtr = 0;
    m_turret.getEndpoint = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
    if (!m_turret.getEndpoint && loopCtr > 10) {
      m_turret.targetAngle = m_turret.endpoint;
      m_turret.getEndpoint = false;
    }

    m_turret.goToPositionMotionMagic(m_turret.endpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.getEndpoint = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.atTargetAngle() && loopCtr > 10;
  }
}
