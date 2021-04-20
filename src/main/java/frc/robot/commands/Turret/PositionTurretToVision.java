// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

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
  private int loopCtr;

  private int visionFoundCounter;
  private boolean targetSeen;

  public PositionTurretToVision(RevTurretSubsystem turret, LimeLight limelight, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;
    m_originalTarget = position;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_originalTarget;
    m_turret.visionCorrection = 0;
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

    if (m_limelight.getIsTargetFound() && visionFoundCounter < 5) {
      visionFoundCounter++;
    }
    if (visionFoundCounter >= 5)
      targetSeen = true;

    if (!m_limelight.getIsTargetFound() && targetSeen) {
      visionFoundCounter--;
    }

    if (targetSeen && visionFoundCounter <= 0) {
      targetSeen = false;
    }

    if (targetSeen)
      visionFoundAngle = m_turret.getAngle() + m_limelight.getdegVerticalToTarget();

    m_endpoint = visionFoundAngle;

    m_turret.goToPositionSmartMotion(m_endpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turret.atTargetAngle() && Math.abs(m_turret.getSpeed()) < .1) && loopCtr > 4;
  }
}
