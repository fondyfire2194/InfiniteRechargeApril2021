// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private double m_position;

  private double m_endpoint;

  private int n;

  private boolean toEnd;

  public PositionTilt(RevTiltSubsystem tilt) {
    m_tilt = tilt;
    m_position = m_tilt.getHeightInches();
    toEnd = false;
    addRequirements(m_tilt);
  }

  public PositionTilt(RevTiltSubsystem tilt, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_position = position;
    toEnd = true;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("SMPos", m_position);
    m_endpoint = m_position / HoodedShooterConstants.TILT_DEG_PER_ENCODER_REV;
    SmartDashboard.putNumber("SMEP", m_endpoint);
    // m_tilt.visionCorrection=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_tilt.goToPosition(m_endpoint);
    n++;
    SmartDashboard.putNumber("N", n);
    m_tilt.goToPositionMotionMagic(m_endpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toEnd && m_tilt.isAtHeight(m_position, 10);
  }
}
