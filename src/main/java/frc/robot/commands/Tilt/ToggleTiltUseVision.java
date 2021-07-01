// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleTiltUseVision extends InstantCommand {
  private RevTiltSubsystem m_tilt;

  public ToggleTiltUseVision(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_tilt.notUseVision)
      m_tilt.notUseVision = false;
    else
      m_tilt.notUseVision = true;
  }
}
