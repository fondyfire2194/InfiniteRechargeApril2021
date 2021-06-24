/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;

public class DriveStraightJoystick extends CommandBase {
  /**
   * Creates a new DriveStraightJoystick.
   */
  private final RevDrivetrain m_drive;
  private double startAngle;
  private DoubleSupplier forward;

  public DriveStraightJoystick(RevDrivetrain drive, DoubleSupplier forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    this.forward = forward;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = m_drive.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drive.arcadeDrive(forward.getAsDouble(), (-m_drive.getYaw() - startAngle) * Pref.getPref("dRStKp"));
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
