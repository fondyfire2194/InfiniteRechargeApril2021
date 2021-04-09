// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.ControlPanelSubsystem;

/** Add your docs here. */
public class SetLookForColor extends InstantCommand {
  /** Add your docs here. */
  private final ControlPanelSubsystem m_panel;
  private boolean m_on;

  public SetLookForColor(ControlPanelSubsystem panel, boolean on) {

    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_panel = panel;
    m_on = on;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    m_panel.setLookForColor(m_on);
  }
}
