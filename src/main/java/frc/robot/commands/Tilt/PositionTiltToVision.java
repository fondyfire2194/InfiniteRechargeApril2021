// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTiltToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_originalTarget;
  private boolean targetSeen;
  private double m_endpoint;
  private int visionFoundCounter;
  private double visionFoundAngle;
  private boolean endIt;
  private int loopCtr;
  private double motorDegrees;

  private double limelightVerticalAngle;

  private final int filterCount = 5;

  public PositionTiltToVision(RevTiltSubsystem tilt, LimeLight limelight, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_originalTarget = position;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_originalTarget;
    m_tilt.visionCorrection = 0;
    m_tilt.targetAngle = m_endpoint;
    motorDegrees = (m_endpoint - m_tilt.tiltMinAngle);
    loopCtr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;
    targetSeen = m_limelight.getIsTargetFound();

    if (targetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (!m_tilt.validTargetSeen && visionFoundCounter >= filterCount)
      m_tilt.validTargetSeen = true;

    if (!targetSeen && m_tilt.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && visionFoundCounter < 0) {
      m_tilt.validTargetSeen = false;
      visionFoundCounter = 0;
    }

    if (targetSeen && m_tilt.validTargetSeen) {
      limelightVerticalAngle = m_limelight.getdegVerticalToTarget();

      visionFoundAngle = m_tilt.getAngle() + limelightVerticalAngle + m_tilt.targetVerticalOffset;

      m_endpoint = visionFoundAngle;

      m_tilt.targetAngle = m_endpoint;
    }

    m_tilt.goToPosition(motorDegrees);

    endIt = m_tilt.validTargetSeen && visionFoundCounter > 5
        || m_tilt.atTargetAngle() && loopCtr > 5 || loopCtr > 250;

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
