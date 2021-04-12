// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class ResetTurretAngle extends CommandBase {
  /** Creates a new ResetTurretAngle. */
  private final RevTurretSubsystem m_turret;

  public ResetTurretAngle(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.resetAngle(0);
    m_turret.targetAngle = 0;
    m_turret.setSoftwareLimits();
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_turret.setDefaultCommand(new PositionHoldTurret(m_turret));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.getAngle() == 0 && m_turret.getSoftwareLimitsEnabled();
  }
}
