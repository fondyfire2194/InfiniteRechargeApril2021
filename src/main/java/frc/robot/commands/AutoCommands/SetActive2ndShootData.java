// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShootData;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetActive2ndShootData extends InstantCommand {
  private double[] m_activeSet;

  public SetActive2ndShootData(double[] activeSet) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_activeSet = activeSet;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShootData.activeValues = m_activeSet;
  }
}
