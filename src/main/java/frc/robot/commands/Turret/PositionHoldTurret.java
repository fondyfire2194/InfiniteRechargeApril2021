// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This is the default command. It will lock onto the vision target if seen otherwise it will hold where it is
 * 
 * It teleop the turret can be set by the driver to either straight ahead or slightly left.
 * 
 * The tilt will be set to minimum angle if driving straight.
 * The further away the target is, the lower the tilt needs to be so it should pick up the target and lift as the robot gets closer.
 * 
 *  If using the trench then the turret will be set slightly laeft and the tilt higher. The target wont be seen as quickly.
 * 
 * The driver can move the tilt up t look for the target if it isn't picked up.
 */

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionHoldTurret extends CommandBase {
  /** Creates a new Positionturret. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private final RevShooterSubsystem m_shooter;
  private boolean targetSeen;
  private double cameraHorizontalError;
  private int visionFoundCounter;
  private final int filterCount = 3;
  private double deadband = .01;
  private double lastHorizontalError;

  public PositionHoldTurret(RevTurretSubsystem turret, RevShooterSubsystem shooter, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_shooter = shooter;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.programRunning = 1;
    if (m_turret.validTargetSeen && m_limelight.useVision)
      visionFoundCounter = filterCount;
    else
      visionFoundCounter = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_limelight.useVision)
      visionFoundCounter = 0;

    targetSeen = m_limelight.getIsTargetFound() && (m_limelight.useVision);

    if (targetSeen && m_turret.validTargetSeen) {
      cameraHorizontalError = m_limelight.getdegRotationToTarget();
      m_turret.adjustedCameraError = cameraHorizontalError
          - (m_turret.targetHorizontalOffset + m_turret.driverHorizontalOffsetDegrees + m_turret.testHorOffset);
      m_limelight.setHorizontalOffset(
          -(m_turret.targetHorizontalOffset + m_turret.driverHorizontalOffsetDegrees + m_turret.testHorOffset));

    } else {
      cameraHorizontalError = 0;
      m_turret.adjustedCameraError = 0;
      m_limelight.setHorizontalOffset(0);
    }

    if (Math.abs(m_turret.adjustedCameraError) < deadband)
      m_turret.adjustedCameraError = 0;

    if (targetSeen && visionFoundCounter < filterCount) {
      visionFoundCounter++;
    }

    if (m_limelight.useVision && visionFoundCounter >= filterCount)
      m_turret.validTargetSeen = true;

    if (!targetSeen && m_turret.validTargetSeen) {
      visionFoundCounter--;
    }

    if (!m_limelight.useVision || !targetSeen && visionFoundCounter <= 0) {
      m_turret.validTargetSeen = false;
      visionFoundCounter = 0;
      cameraHorizontalError = 0;
    }
    if (!m_shooter.shotInProgress)
      lastHorizontalError = -m_turret.adjustedCameraError;
    if (!m_turret.validTargetSeen) {

      m_turret.goToPositionMotionMagic(m_turret.targetAngle);
    }

    else {
      m_turret.lockTurretToVision(lastHorizontalError);
      m_turret.targetAngle = m_turret.getAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.targetAngle = m_turret.getAngle();
    m_turret.validTargetSeen = false;
    m_turret.visionOnTarget = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
