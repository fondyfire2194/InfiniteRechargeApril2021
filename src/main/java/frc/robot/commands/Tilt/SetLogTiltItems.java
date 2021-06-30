// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLogTiltItems extends InstantCommand {
  private RevTiltSubsystem m_tilt;
  private boolean m_state;

  public SetLogTiltItems(RevTiltSubsystem tilt, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.logTiltItems = m_state;
  }
}
