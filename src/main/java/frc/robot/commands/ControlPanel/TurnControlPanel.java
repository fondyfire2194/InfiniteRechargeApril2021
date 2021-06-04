/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class TurnControlPanel extends CommandBase {
  /**
   * Creates a new RotateControlPanel.
   * 
   */

  private ControlPanelSubsystem m_cp;
  private int startColor;
  private int currentColor;


  private int colorsToPass = 9;
  private boolean redSeen;
  private int lastColor;

  public TurnControlPanel(ControlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cp);
    m_cp = cp;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startColor = m_cp.colorNumberFiltered;
    lastColor = startColor;
    m_cp.colorsPassed = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cp.turnWheelMotor();
    currentColor = m_cp.colorNumberFiltered;

    if (!redSeen && currentColor == 3) {
      m_cp.colorsPassed++;
    }
    if (currentColor == 3) {
      redSeen = true;
    }
    if (currentColor == 4) {
      redSeen = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cp.stopWheelMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_cp.colorsPassed >= colorsToPass;
  }
}
