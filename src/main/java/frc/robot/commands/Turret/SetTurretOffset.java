// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTurretOffset extends InstantCommand {
  private RevTurretSubsystem m_turret;
  private double m_offset;

  public SetTurretOffset(RevTurretSubsystem turret, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_offset = offset;
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.targetHorizontalOffset = m_offset;
  }
}
