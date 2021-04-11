// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class PositionNumberRevs extends CommandBase {
  /** Creates a new PositionToColor. */
  private final ControlPanelSubsystem m_panel;
  private int m_numberRevs;
  private double m_speed;
  private int revsMade;
  private int lastColor;

  public PositionNumberRevs(ControlPanelSubsystem panel, int numberRevs, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_panel = panel;
    m_numberRevs = numberRevs;
    m_speed = speed;
    addRequirements(m_panel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_panel.revsDone = 0;
    lastColor = m_panel.colorNumberFiltered;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_panel.lowerArm();
    m_panel.lookForColor = true;
    m_panel.turnWheelMotor(m_speed);
    if (m_panel.colorNumberFiltered != lastColor) {
      lastColor = m_panel.colorNumberFiltered;
      m_panel.revsDone++;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_panel.turnWheelMotor(0);
    m_panel.raiseArm();
    m_panel.lookForColor = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_panel.revsDone == m_numberRevs;
  }
}
