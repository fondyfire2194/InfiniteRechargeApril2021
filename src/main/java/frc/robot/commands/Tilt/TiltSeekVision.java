// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class TiltSeekVision extends CommandBase {
  /** Creates a new PositionTilttoVision. */

  private final RevTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_position;
  private boolean targetSeen;
  private double m_endpoint;
  private int visionFoundCounter;
  private boolean endIt;
  private int loopCtr;
  private double motorDegrees;

  private double m_offset;

  private final int filterCount = 5;

  private int tryCounter;

  private boolean targetFound;
  private boolean noTargetFound;
  private boolean failedToFind;

  public TiltSeekVision(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_position;
    m_limelight.useVision = true;
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_tilt.targetAngle = m_tilt.tiltMinAngle;
    m_tilt.targetVerticalOffset = m_offset;
    motorDegrees = (m_tilt.tiltMaxAngle - m_endpoint);
    loopCtr = 0;
    tryCounter = 0;
    failedToFind = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;

    targetSeen = m_limelight.useVision && m_limelight.getIsTargetFound();

    if (targetSeen && !m_tilt.validTargetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (!m_tilt.validTargetSeen && visionFoundCounter >= filterCount) {
      m_tilt.validTargetSeen = true;
    }

    if (!targetSeen && m_tilt.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && visionFoundCounter < 0) {
      m_tilt.validTargetSeen = false;
      visionFoundCounter = 0;
    }

    m_tilt.goToPositionMotionMagic(motorDegrees);

    noTargetFound = m_tilt.atTargetAngle() && loopCtr > 5;

    if (noTargetFound && m_endpoint == m_tilt.tiltMinAngle) {
      m_endpoint = m_tilt.tiltMaxAngle;
      loopCtr = 0;
    }

    targetFound = m_tilt.validTargetSeen;

    failedToFind = m_endpoint == m_tilt.tiltMaxAngle && (m_tilt.atTargetAngle() && loopCtr > 5);
    endIt = targetFound || failedToFind;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!endIt)
      m_tilt.targetAngle = m_tilt.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
