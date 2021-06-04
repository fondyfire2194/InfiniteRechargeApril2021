// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class PositionToColor extends CommandBase {
  /** Creates a new PositionToColor. */
  private final ControlPanelSubsystem m_panel;
  private double m_speed;
  private int loopCount;

  public PositionToColor(ControlPanelSubsystem panel, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_panel = panel;
    m_speed = speed;
    addRequirements(m_panel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopCount = 0;
    m_panel.lookForColor = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_panel.lowerArm();
    m_panel.turnWheelMotor();

    if (m_panel.colorNumberFiltered == m_panel.gameColorNumber) {
      loopCount++;
    } else {
      loopCount = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_panel.stopWheelMotor();
    m_panel.raiseArm();
    m_panel.lookForColor = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopCount > 10;// || m_gameColorNumber == 0;
  }
}
