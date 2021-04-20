// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTiltToVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_originalTarget;
private boolean targetSeen;
  private double m_endpoint;
private int visionFoundCounter;
  private int loopCtr;
  private double visionFoundAngle;

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
    loopCtr=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
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
      visionFoundAngle = m_tilt.getAngle() + m_limelight.getdegVerticalToTarget();

    m_endpoint = visionFoundAngle;

    m_tilt.goToPositionSmartMotion(m_endpoint);

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tilt.atTargetAngle() && loopCtr > 10;
  }
}
