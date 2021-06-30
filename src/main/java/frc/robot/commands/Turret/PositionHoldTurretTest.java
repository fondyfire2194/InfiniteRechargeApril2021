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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RevTurretSubsystem;

public class PositionHoldTurretTest extends CommandBase {
  /** Creates a new Positionturret. */

  private final RevTurretSubsystem m_turret;

  public PositionHoldTurretTest(RevTurretSubsystem turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.programRunning = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("TTL", m_turret.testLock);
    if (!m_turret.testLock) {

      m_turret.goToPositionMotionMagic(m_turret.targetAngle);
    }

    else {
      m_turret.lockTurretToThrottle(m_turret.testLockFromThrottle * 10);

      SmartDashboard.putNumber("TTLFRTH", m_turret.testLockFromThrottle);

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
