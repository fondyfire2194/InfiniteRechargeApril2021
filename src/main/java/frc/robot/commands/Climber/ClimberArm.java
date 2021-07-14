// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ClimberArm extends CommandBase {

  private ClimberSubsystem m_climber;
  private boolean m_lower;
  private double m_startTime;

  public ClimberArm(ClimberSubsystem climber, boolean lower) {
    m_climber = climber;
    m_lower = lower;
  
  }

  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override

  public void execute() {
    if (m_lower) {
      m_climber.lowerArm();
    } else {
      m_climber.raiseArm();
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
