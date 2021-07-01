// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleTurretUseVision extends InstantCommand {
  private RevTurretSubsystem m_turret;

  public ToggleTurretUseVision(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_turret.notUseVision)
      m_turret.notUseVision = false;
    else
      m_turret.notUseVision = true;
  }
}
