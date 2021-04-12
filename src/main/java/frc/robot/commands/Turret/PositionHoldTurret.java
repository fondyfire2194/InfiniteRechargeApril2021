// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionHoldTurret extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;

  private double m_position;

  public PositionHoldTurret(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_position = m_turret.targetAngle;
    m_turret.visionCorrection = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_position = m_turret.targetAngle;
    m_turret.goToPositionMotionMagic(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
