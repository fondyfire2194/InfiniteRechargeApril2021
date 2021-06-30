// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This is the default command. It will lock onto the vision target if seen otherwise it will hold where it is
 * 
 * It teleop the tilt can be set by the driver to either straight ahead or slightly left.
 * 
 * The tilt will be set to minimum angle if driving straight.
 * The further away the target is, the lower the tilt needs to be so it should pick up the target and lift as the robot gets closer.
 * 
 *  If using the trench then the tilt will be set slightly laeft and the tilt higher. The target wont be seen as quickly.
 * 
 * The driver can move the tilt up t look for the target if it isn't picked up.
 */

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;


public class PositionHoldTiltTest extends CommandBase {
  /** Creates a new Positiontilt. */

  private final RevTiltSubsystem m_tilt;

  public PositionHoldTiltTest(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;

    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_tilt.programRunning = 1;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("TTiL", m_tilt.testLock);

    m_tilt.motorEndpointDegrees = m_tilt.tiltMaxAngle - m_tilt.targetAngle;

    if (!m_tilt.testLock) {

      m_tilt.goToPositionMotionMagic(m_tilt.motorEndpointDegrees);
    }

    else {
      m_tilt.lockTiltToThrottle(m_tilt.testLockFromThrottle * 2);

      SmartDashboard.putNumber("TTILFRTH", m_tilt.testLockFromThrottle);

      m_tilt.targetAngle = m_tilt.getAngle();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.targetAngle = m_tilt.getAngle();
    m_tilt.validTargetSeen = false;
    m_tilt.visionOnTarget = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
