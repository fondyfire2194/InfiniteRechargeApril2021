// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ClimberRatchet extends CommandBase {

  private ClimberSubsystem m_climber;
  private boolean m_lock;
  private double m_startTime;

  public ClimberRatchet(ClimberSubsystem climber, boolean lock) {
    m_climber = climber;
    m_lock = lock;

  }

  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  @Override

  public void execute() {
    if (m_lock) {
      m_climber.lockRatchet();
    } else {
      m_climber.unlockRatchet();
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
