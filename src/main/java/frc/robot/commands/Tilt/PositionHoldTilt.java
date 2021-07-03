// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionHoldTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;
  private final LimeLight m_limelight;
  private final RevShooterSubsystem m_shooter;
  private boolean targetSeen;
  private int visionFoundCounter;
  private double cameraVerticalError;
  private final int filterCount = 3;
  private double deadband = .01;
  private int loopctr;
  private double lastVerticalError;

  public PositionHoldTilt(RevTiltSubsystem tilt, RevShooterSubsystem shooter, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_shooter = shooter;
    m_limelight = limelight;
    visionFoundCounter = 0;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.programRunning = 1;
    if (!m_limelight.useVision)
      visionFoundCounter = 0;

    if (m_tilt.validTargetSeen && m_limelight.useVision)
      visionFoundCounter = filterCount;
    else
      visionFoundCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (visionFoundCounter < 0)
      visionFoundCounter = 0;
    if (visionFoundCounter > filterCount)
      visionFoundCounter = filterCount;

    loopctr++;

    if (!m_limelight.useVision)
      visionFoundCounter = 0;

    targetSeen = m_limelight.getIsTargetFound() && m_limelight.useVision;// && m_tilt.tiltUseVision;

    if (targetSeen && m_tilt.validTargetSeen) {

      cameraVerticalError = m_limelight.getdegVerticalToTarget();

      m_tilt.adjustedVerticalError = cameraVerticalError
          + (m_tilt.targetVerticalOffset + m_tilt.driverVerticalOffsetDegrees + m_tilt.testVerticalOffset);

      m_limelight.setVerticalOffset(
          -(m_tilt.targetVerticalOffset + m_tilt.driverVerticalOffsetDegrees + m_tilt.testVerticalOffset));

    } else {
      cameraVerticalError = 0;
      m_tilt.adjustedVerticalError = 0;
      m_limelight.setVerticalOffset(0);
    }

    if (Math.abs(m_tilt.adjustedVerticalError) < deadband)
      m_tilt.adjustedVerticalError = 0;

    if (targetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (m_limelight.useVision && visionFoundCounter >= filterCount)
      m_tilt.validTargetSeen = true;

    if (!targetSeen && m_tilt.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!m_limelight.useVision || !targetSeen && visionFoundCounter < 0) {
      m_tilt.validTargetSeen = false;
      visionFoundCounter = 0;
      cameraVerticalError = 0;
    }

    m_tilt.motorEndpointDegrees = m_tilt.tiltMaxAngle - m_tilt.targetAngle;

    if (!m_shooter.shotInProgress)
      lastVerticalError = m_tilt.adjustedVerticalError;

    if (!m_tilt.validTargetSeen) {
      m_tilt.goToPositionMotionMagic(m_tilt.motorEndpointDegrees);
    } else {
      m_tilt.lockTiltToVision(lastVerticalError);
      m_tilt.targetAngle = m_tilt.getAngle();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.targetAngle = m_tilt.getAngle();
    m_tilt.validTargetSeen = false;
    m_tilt.visionOnTarget = false;
    visionFoundCounter = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
