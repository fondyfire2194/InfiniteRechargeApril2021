// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTeleopShooterSpeed extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private double m_mps;
  public SetTeleopShooterSpeed(RevShooterSubsystem shooter, double mps) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter=shooter;
    m_mps = mps;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.teleopSetupShooterSpeed = m_mps;
  }
}
