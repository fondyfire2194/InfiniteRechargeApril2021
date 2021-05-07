// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class PositionTurretInc extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevTurretSubsystem m_turret;
  private double m_increment;
  private int loopCtr;
  private boolean timedOut;
  private double m_startTime;

  public PositionTurretInc(RevTurretSubsystem turret, double increment) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_increment = increment;

    addRequirements(m_turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.targetAngle += m_increment;
    loopCtr = 0;
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.goToPositionMotionMagic(m_turret.targetAngle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (timedOut)
      m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_turret.atTargetAngle() && Timer.getFPGATimestamp() > m_startTime + .02
        && Math.abs(m_turret.getSpeed()) < .1) || Timer.getFPGATimestamp() > m_startTime + 2;
  }
}
