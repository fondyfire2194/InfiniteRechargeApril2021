// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RearIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class IntakeArmLower extends CommandBase {

  private RearIntakeSubsystem m_intake;

  public IntakeArmLower(RearIntakeSubsystem intake) {
    m_intake = intake;

  }

  public void initialize() {

  }

  @Override

  public void execute() {

    m_intake.lowerArm();

  }

  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
