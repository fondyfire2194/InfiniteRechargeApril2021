// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionHoldTilt extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;
  private final LimeLight m_limelight;

  private boolean targetSeen;
  private double m_endpoint;
  private int visionFoundCounter;
  private double visionFoundAngle;
  private double limelightVerticalAngle;
  private final int filterCount = 3;
  private double deadband = .1;
  private int activeGainSlot;

  public PositionHoldTilt(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_limelight = limelight;

    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_tilt.targetAngle;
    if (m_tilt.validTargetSeen)
      visionFoundCounter = filterCount;
    else
      visionFoundCounter = 0;
    activeGainSlot = m_tilt.POSITION_SLOT;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetSeen = m_limelight.getIsTargetFound();
    if (targetSeen && m_tilt.validTargetSeen) {
      limelightVerticalAngle = m_limelight.getdegVerticalToTarget();
      m_tilt.adjustedTargetAngle = limelightVerticalAngle + m_tilt.adjustedTargetAngle;
      m_limelight.setVerticalOffset(m_tilt.targetVerticalOffset);
    } else {
      limelightVerticalAngle = 0;
      m_tilt.adjustedTargetAngle = 0;
      m_limelight.setVerticalOffset(0);
    }
    if (Math.abs(m_tilt.adjustedTargetAngle) < deadband)
      m_tilt.adjustedTargetAngle = 0;
    if (targetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (visionFoundCounter >= filterCount)
      m_tilt.validTargetSeen = true;

    if (!targetSeen && m_tilt.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && visionFoundCounter < 0) {
      m_tilt.validTargetSeen = false;
      visionFoundCounter = 0;
      limelightVerticalAngle = 0;
    }

    if (RobotBase.isReal() && targetSeen) {
      visionFoundAngle = m_tilt.getAngle() + m_tilt.adjustedTargetAngle;
      m_endpoint = visionFoundAngle;
      m_tilt.targetAngle = m_endpoint;
      activeGainSlot = m_tilt.VISION_SLOT;
    }

    double motorTurns = m_endpoint - m_tilt.tiltMinAngle;
    m_tilt.positionTilt(motorTurns, activeGainSlot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.targetAngle = m_tilt.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
