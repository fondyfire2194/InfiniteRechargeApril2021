// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class LockTiltToVision extends CommandBase {
  /** Creates a new LockTiltToVision. */
  private RevTiltSubsystem m_tilt;
  private LimeLight m_limelight;
  private int visionFoundCounter;
  private int filterCount = 3;

  private boolean targetSeen;
  private double deadband = .01;
  private double cameraVerticalError;

  public LockTiltToVision(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (visionFoundCounter < 0)
      visionFoundCounter = 0;
    if (visionFoundCounter > filterCount)
      visionFoundCounter = filterCount;
    

    if (!m_limelight.useVision)
      visionFoundCounter = 0;

    targetSeen = m_limelight.getIsTargetFound() && m_limelight.useVision;

    if (targetSeen && m_tilt.validTargetSeen) {

      cameraVerticalError = m_limelight.getdegVerticalToTarget();

      m_tilt.adjustedVerticalError = cameraVerticalError + m_tilt.targetVerticalOffset
          + m_tilt.driverVerticalOffsetDegrees + m_tilt.testVerticalOffset;

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

    double motorTurns = m_tilt.tiltMaxAngle - m_tilt.targetAngle;
    m_tilt.motorEndpointDegrees = motorTurns;

    if (!m_tilt.validTargetSeen) {
      m_tilt.goToPositionMotionMagic(motorTurns);
    } else {
      m_tilt.lockTiltToVision(cameraVerticalError);
      m_tilt.targetAngle = m_tilt.getAngle();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tilt.getLockAtTarget();
  }
}
