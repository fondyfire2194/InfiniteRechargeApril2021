// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ControlPanelArm extends CommandBase {

  private ControlPanelSubsystem m_panel;
  private boolean m_lower;
  private double m_startTime;

  public ControlPanelArm(ControlPanelSubsystem panel, boolean lower) {
    m_panel = panel;
    m_lower = lower;
    addRequirements(m_panel);
  }

  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override

  public void execute() {
    if (m_lower) {
      m_panel.lowerArm();
    } else {
      m_panel.raiseArm();
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + .5;

  }
}
