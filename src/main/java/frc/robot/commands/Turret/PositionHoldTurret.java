// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This s the default command. It will lock onto the vision target if seen otherwise it will hold where it is
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

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionHoldTurret extends CommandBase {
  /** Creates a new Positionturret. */

  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;
  private boolean targetSeen;
  private boolean targetWasSeen;
  private double limelightHorizontalAngle;
  private int visionFoundCounter;
  private final int filterCount = 3;
  private double visionFoundAngle;
  private double m_endpoint;

  public PositionHoldTurret(RevTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_limelight = limelight;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
