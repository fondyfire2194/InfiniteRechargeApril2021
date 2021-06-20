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
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTiltToVision extends CommandBase {
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

  private final int filterCount = 5;

  public PositionTiltToVision(RevTiltSubsystem tilt, LimeLight limelight, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_position = position;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_position;

    m_limelight.useVision = true;
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_tilt.targetAngle = m_endpoint;
    m_tilt.targetVerticalOffset = 0;
    m_tilt.driverVerticalOffset = 0;
    if (m_endpoint < HoodedShooterConstants.TILT_MIN_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MIN_ANGLE;
    if (m_endpoint > HoodedShooterConstants.TILT_MAX_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MAX_ANGLE;
    motorDegrees = (m_tilt.tiltMaxAngle - m_endpoint);
    loopCtr = 0;
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

    endIt = m_limelight.getHorOnTarget(5) || m_tilt.atTargetAngle() && loopCtr > 5 || loopCtr > 250;

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
    return endIt || m_limelight.useVision;
  }
}
