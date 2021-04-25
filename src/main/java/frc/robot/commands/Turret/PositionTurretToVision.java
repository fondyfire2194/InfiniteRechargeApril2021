// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Used in auto to find vision target then end and the default Hold command will take over and lock onto vision
 * 
 * 
 */

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private double m_originalTarget;

  private double m_endpoint;
  private double visionFoundAngle;
  private final int filterCount = 3;
  private double limelightHorizontalAngle;
  private int visionFoundCounter;
  private boolean targetSeen;
  private boolean targetWasSeen;
  boolean endIt;

  public PositionTurretToVision(RevTurretSubsystem turret, LimeLight limelight, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_originalTarget = position;
    m_turret.targetAngle = m_originalTarget;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_originalTarget;
    m_turret.visionCorrection = 0;
    targetSeen = false;
    visionFoundCounter = 0;
    if (RobotBase.isSimulation()) {
      targetSeen = true;
      m_endpoint += Math.random();
      m_turret.targetAngle = m_endpoint;
    }
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
    if (RobotBase.isReal()) {
      targetSeen = m_limelight.getIsTargetFound();
      if (targetSeen && targetWasSeen)
        limelightHorizontalAngle = m_limelight.getdegVerticalToTarget();
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
      limelightHorizontalAngle = 0;
    }

    if (RobotBase.isReal() && targetSeen) {
      visionFoundAngle = m_turret.getAngle() + limelightHorizontalAngle + m_turret.targetHorizontalOffset;
      m_endpoint = visionFoundAngle;
      m_turret.targetAngle = m_endpoint;
    }
    endIt = targetSeen && visionFoundCounter > 5;
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
