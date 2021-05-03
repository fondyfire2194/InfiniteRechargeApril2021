// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class TiltTune extends CommandBase {
  /** Creates a new TilTune. */
  private final RevTiltSubsystem m_tilt;

  public TiltTune(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("P Gain", m_tilt.kP);
    SmartDashboard.putNumber("I Gain", m_tilt.kI);
    SmartDashboard.putNumber("D Gain", m_tilt.kD);
    SmartDashboard.putNumber("I Zone", m_tilt.kIz);
    SmartDashboard.putNumber("Feed Forward", m_tilt.kFF);
    SmartDashboard.putNumber("Max Output", m_tilt.kMaxOutput);
    SmartDashboard.putNumber("Min Output", m_tilt.kMinOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tilt.kP = SmartDashboard.getNumber("P Gain", 0);
    m_tilt.kI = SmartDashboard.getNumber("I Gain", 0);
    m_tilt.kD = SmartDashboard.getNumber("D Gain", 0);
    m_tilt.kIz = SmartDashboard.getNumber("I Zone", 0);
    m_tilt.kFF = SmartDashboard.getNumber("Feed Forward", 0);
    m_tilt.kMaxOutput = SmartDashboard.getNumber("Max Output", 0);
    m_tilt.kMinOutput = SmartDashboard.getNumber("Min Output", 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
