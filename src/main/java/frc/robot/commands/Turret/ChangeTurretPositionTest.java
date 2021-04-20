// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeTurretPositionTest extends InstantCommand {
  private final RevTurretSubsystem m_turret;
  private double m_change;

  public ChangeTurretPositionTest(RevTurretSubsystem turret, double change) {
    // Use addRequirements() here to decl are subsystem dependencies.
    m_turret = turret;
    m_change = change;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Math.abs(m_change) > 5)
      m_change = 0;
    m_turret.targetAngle += m_change;
  }
}
