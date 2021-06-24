// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RearIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ToggleIntakeArmOff extends CommandBase {

  private RearIntakeSubsystem m_intake;
  private boolean m_lower;
  private double m_startTime;

  public ToggleIntakeArmOff(RearIntakeSubsystem intake) {
    m_intake = intake;

  }

  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override

  public void execute() {
    if (m_intake.getArmOff()) {
      m_intake.lowerArm();
    } else {
      m_intake.armSolenoidOff();
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
