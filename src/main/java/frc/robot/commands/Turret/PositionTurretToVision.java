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
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private double m_originalTarget;
  private int loopCtr;
  private double m_endpoint;
  private double visionFoundAngle;
  private final int filterCount = 3;
  private double limelightHorizontalAngle;
  private int visionFoundCounter;
  private boolean targetSeen;
  boolean endIt;
  private double m_offset;

  public PositionTurretToVision(RevTurretSubsystem turret, LimeLight limelight, double position, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_originalTarget = position;
    m_turret.targetAngle = m_originalTarget;
    m_offset = offset;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_originalTarget;
    m_turret.targetHorizontalOffset = m_offset;
    targetSeen = false;
    visionFoundCounter = 0;
    loopCtr = 0;
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
    }

    if (!targetSeen && m_turret.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!targetSeen && m_turret.validTargetSeen && visionFoundCounter < 0) {
      visionFoundCounter = 0;
      m_turret.validTargetSeen = false;
      limelightHorizontalAngle = 0;
    }

    m_turret.goToPositionMotionMagic(m_turret.targetAngle);

    endIt = m_turret.validTargetSeen || (m_turret.atTargetAngle() && loopCtr > 5) || loopCtr > 250;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!endIt)
      m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt;
  }
}
