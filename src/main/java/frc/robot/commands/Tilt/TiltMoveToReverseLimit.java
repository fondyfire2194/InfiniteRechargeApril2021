/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.RevTiltSubsystem;

public class TiltMoveToReverseLimit extends CommandBase {
  /**
   * Creates a new TiltMoveToReverseLimit.
   */
  private int simCter;
  private final RevTiltSubsystem tilt;

  public TiltMoveToReverseLimit(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    simCter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!tilt.positionResetDone)
      tilt.moveManually(-.2);
    SmartDashboard.putBoolean("CMDTI2SWLRng", true);
    if (Robot.isSimulation())
      simCter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("CMDTI2SWLRng", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tilt.positionResetDone || tilt.m_reverseLimit.get() || simCter > 25;
  }
}
