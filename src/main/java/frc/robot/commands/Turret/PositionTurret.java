// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurret extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private double m_endpoint;
  private int loopCtr;
  private boolean endIt;

  public PositionTurret(RevTurretSubsystem turret, double endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_endpoint = endpoint;

    addRequirements(m_turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.targetAngle = m_endpoint;
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.goToPositionMotionMagic(m_turret.targetAngle);
    loopCtr++;
    endIt = m_turret.atTargetAngle() && loopCtr > 10 && Math.abs(m_turret.getSpeed()) < 1;// ||
                                                                                          // !m_tilt.positionResetDone;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (loopCtr > 10 && !endIt)
      m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
