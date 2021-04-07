/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class RotateControlPanel extends CommandBase {
  /**
   * Creates a new RotateControlPanel.
   * 
   */

  private ControlPanelSubsystem cp;
  private int startColor;
  private int currentColor;
  private int lastColor;
  private int colorsPassed;
  private int colorsToPass = 9;
  private boolean redSeen;

  public RotateControlPanel(ControlPanelSubsystem cp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(cp);
    this.cp = cp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startColor = cp.colorNumberFiltered;
    lastColor = startColor;
    colorsPassed = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cp.turnWheelMotor(.1);
    currentColor = cp.colorNumberFiltered;

if (!redSeen && currentColor == 3) {
      colorsPassed++;
    }
    if (currentColor == 3) {
      redSeen = true;
    }
    if (currentColor == 4) {
      redSeen = false;
    }

    

    // if (currentColor != lastColor)

    // {
    //   colorsPassed++;
    //   lastColor = currentColor;
    // }
    Shuffleboard.getTab("ControlPanel").add("ColorsPassed", colorsPassed);
    Shuffleboard.getTab("ControlPanel").add("ColorNow", currentColor);
    Shuffleboard.getTab("ControlPanel").add("ColorsLast", lastColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cp.turnWheelMotor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return colorsPassed >= colorsToPass;
  }
}
