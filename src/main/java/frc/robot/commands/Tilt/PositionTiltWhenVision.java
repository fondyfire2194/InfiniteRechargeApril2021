// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class PositionTiltWhenVision extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTiltSubsystem m_tilt;

  private final LimeLight m_limelight;

  private double m_originalTarget;
  private boolean targetSeen;
  private double m_endpoint;
  private int visionFoundCounter;
  private int loopCtr;
  private double visionFoundAngle;
  private boolean endIt;

  public PositionTiltWhenVision(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_originalTarget = m_tilt.getAngle();
    m_limelight = limelight;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endpoint = m_originalTarget;
    m_tilt.visionCorrection = 0;
    m_tilt.targetAngle = m_endpoint;
    loopCtr = 0;
    if (RobotBase.isSimulation()) {
      targetSeen = true;
      m_endpoint += Math.random();
      m_tilt.targetAngle = m_endpoint;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCtr++;

    if (RobotBase.isReal())
      targetSeen = m_limelight.getIsTargetFound();

    if (targetSeen && visionFoundCounter < 5) {
      visionFoundCounter++;
    }
    if (visionFoundCounter >= 5)
      targetSeen = true;

    if (!targetSeen && targetSeen) {
      visionFoundCounter--;
    }

    if (targetSeen && visionFoundCounter <= 0) {
      targetSeen = false;
    }

    if (RobotBase.isReal() && targetSeen) {
      visionFoundAngle = m_tilt.getAngle() + m_limelight.getdegVerticalToTarget() + m_tilt.targetVerticalOffset;
      m_endpoint = visionFoundAngle;
      m_tilt.targetAngle = m_endpoint;
    }

    m_tilt.goToPositionMotionMagic(m_endpoint);

    endIt = (m_limelight.getVertOnTarget() && m_limelight.getHorOnTarget() && Math.abs(m_tilt.getSpeed()) < .1)
        && loopCtr > 4;

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