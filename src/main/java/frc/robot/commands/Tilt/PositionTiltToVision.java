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

  private boolean targetWasSeen;

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

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotBase.isReal()) {
      targetSeen = m_limelight.getIsTargetFound();
      if (targetSeen && targetWasSeen)
        limelightVerticalAngle = m_limelight.getdegVerticalToTarget();
    }

    if (targetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (visionFoundCounter >= filterCount)
      targetWasSeen = true;

    if (!targetSeen && targetWasSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && visionFoundCounter <= 0) {
      targetWasSeen = false;
      visionFoundCounter = 0;
      limelightVerticalAngle = 0;
    }

    if (RobotBase.isReal() && targetSeen) {
      visionFoundAngle = m_tilt.getAngle() + limelightVerticalAngle + m_tilt.targetVerticalOffset;
      m_endpoint = visionFoundAngle;
      m_tilt.targetAngle = m_endpoint;
    }
    m_tilt.goToPositionMotionMagic(m_endpoint);
    endIt = targetSeen && visionFoundCounter > 5;

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
