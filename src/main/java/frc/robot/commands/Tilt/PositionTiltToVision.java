// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTiltToVision extends CommandBase {
  /** Creates a new PositionTilttoVision. */

  private final RevTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_endpoint;
  private boolean targetSeen;

  private int visionFoundCounter;
  private boolean endIt;
  private int loopCtr;
  private final int filterCount = 5;
  private int correctionCtr;

  public PositionTiltToVision(RevTiltSubsystem tilt, LimeLight limelight, double endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_endpoint = endpoint;
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_limelight.useVision = false;
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_limelight.setLEDMode(LedMode.kpipeLine);
    m_tilt.targetAngle = m_endpoint;
    if (m_endpoint < HoodedShooterConstants.TILT_MIN_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MIN_ANGLE;
    if (m_endpoint > HoodedShooterConstants.TILT_MAX_ANGLE)
      m_endpoint = HoodedShooterConstants.TILT_MAX_ANGLE;
    m_tilt.correctedEndpoint = m_endpoint;
    loopCtr = 0;
    m_tilt.logTrigger = true;
    m_limelight.setVerticalOffset(m_tilt.targetVerticalOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;

    targetSeen = m_limelight.getIsTargetFound();

    if (!targetSeen && m_tilt.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && visionFoundCounter < 0) {
      m_tilt.validTargetSeen = false;
      visionFoundCounter = 0;
    }

    if (targetSeen && !m_tilt.validTargetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (!m_tilt.validTargetSeen && visionFoundCounter >= filterCount) {

      m_tilt.validTargetSeen = true;
    }

    if (m_tilt.validTargetSeen) {

      correctionCtr++;

      if (correctionCtr >= 5) {

        m_tilt.correctedEndpoint = m_tilt.getAngle() + m_limelight.getdegVerticalToTarget();

        m_tilt.targetAngle = m_tilt.correctedEndpoint;

        correctionCtr = 0;
      }
    }

    double motorTurns = m_tilt.tiltMaxAngle - m_tilt.targetAngle;

    m_tilt.motorEndpointDegrees = motorTurns;

    m_tilt.goToPositionMotionMagic(motorTurns);

    endIt = m_limelight.getVertOnTarget(1) || !m_tilt.validTargetSeen && m_tilt.atTargetAngle() && loopCtr > 5
        || loopCtr > 1250;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.targetAngle = m_tilt.getAngle();
    m_tilt.logTrigger = false;
    m_tilt.endTiltFile = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
