// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private int loopCtr;

  private double m_endpoint;

  private boolean endIt;

  private double motorDegrees;

  public PositionTilt(RevTiltSubsystem tilt, double endpoint) {
    m_tilt = tilt;
    m_endpoint = endpoint;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.programRunning = 2;
    if (m_endpoint < HoodedShooterConstants.TILT_MIN_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MIN_ANGLE;
    if (m_endpoint > HoodedShooterConstants.TILT_MAX_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MAX_ANGLE;
    m_tilt.targetAngle = m_endpoint;
    motorDegrees = (m_tilt.tiltMaxAngle - m_endpoint);
    m_tilt.motorEndpointDegrees = motorDegrees;

    if (DriverStation.getInstance().isOperatorControlEnabled())
      m_tilt.logTiltItems = true;

    loopCtr = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;

    m_tilt.goToPositionMotionMagic(motorDegrees);

    endIt = m_tilt.atTargetAngle() && loopCtr > 10 && Math.abs(m_tilt.getSpeed()) < 1;// || !m_tilt.positionResetDone;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (loopCtr > 10 && !endIt)
      m_tilt.targetAngle = m_tilt.getAngle();
    m_tilt.logTiltItems = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
