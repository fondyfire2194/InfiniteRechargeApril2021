// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private double m_endpoint;
  private int loopCtr;
  private final int filterCount = 3;
  private int visionFoundCounter;
  private boolean targetSeen;
  boolean endIt;
  private int correctionCtr;

  public PositionTurretToVision(RevTurretSubsystem turret, LimeLight limelight, double endpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_endpoint = endpoint;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.targetAngle = m_endpoint;
    targetSeen = false;
    visionFoundCounter = 0;
    loopCtr = 0;

    m_limelight.useVision = false;
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_limelight.setLEDMode(LedMode.kpipeLine);
    m_turret.correctedEndpoint = m_endpoint;
  }

  // Called every time the scheduler runs while the command is scheduled.

  /**
   * Watching for camera finding target. When it does, the vision value can be
   * added on to the current position an dbecomes the new target. This will be
   * repeated until the vision error is reduced to 0 +-tolerance;
   * 
   * A filter ignores loss of target for 100ms
   * 
   * 
   */
  @Override
  public void execute() {
    loopCtr++;

    targetSeen = m_limelight.useVision && m_limelight.getIsTargetFound();

    if (targetSeen && !m_turret.validTargetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (!m_turret.validTargetSeen && visionFoundCounter >= filterCount) {

      m_turret.validTargetSeen = true;

      correctionCtr++;

      if (correctionCtr >= 5) {

        m_turret.correctedEndpoint = (m_turret.getAngle() + m_turret.getSpeed() / 50)
            - m_limelight.getdegRotationToTarget();

        m_turret.targetAngle = m_turret.correctedEndpoint;

        correctionCtr = 0;
      }

      if (!targetSeen && m_turret.validTargetSeen) {
        visionFoundCounter--;
      }

      if (!targetSeen && m_turret.validTargetSeen && visionFoundCounter < 0) {
        visionFoundCounter = 0;
        m_turret.validTargetSeen = false;

      }

      m_turret.goToPositionMotionMagic(m_turret.targetAngle);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limelight.getHorOnTarget(1) || m_turret.atTargetAngle() && loopCtr > 5 || loopCtr > 250;
  }
}
