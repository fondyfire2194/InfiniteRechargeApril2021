/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class TiltMoveToReverseLimit extends CommandBase {
  /**
   * Creates a new TiltMoveToReverseLimit.
   */
  private int simCtr;
  private final RevTiltSubsystem m_tilt;
  private double m_startAngle;
  private boolean endIt;

  public TiltMoveToReverseLimit(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    simCtr = 0;
    m_startAngle = m_tilt.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    simCtr++;
    if (!endIt) {
      // if (!m_tilt.positionResetDone) {
      m_tilt.moveManually(-.2);
      // if (Robot.isSimulation())
      // simCtr++;
      // }
    }
    endIt = m_tilt.m_reverseLimit.get() || m_tilt.getAngle() < m_startAngle - 10 || simCtr > 2500;

    if (endIt)
      m_tilt.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.resetAngle(0);
    m_tilt.positionResetDone = true;
    m_tilt.setSoftwareLimits();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endIt && Math.abs(m_tilt.getSpeed()) < .01;
  }
}
