// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ControlPanelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArm extends InstantCommand {
  private final ControlPanelSubsystem m_cp;
  private boolean m_up;

  public MoveArm(ControlPanelSubsystem cp, boolean up) {
    // Use addRequirements() here to declare subsys cp;tem dependencies.
    m_cp = cp;
    m_up = up;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_up)
      m_cp.raiseArm();
    else
      m_cp.lowerArm();
  }
}
