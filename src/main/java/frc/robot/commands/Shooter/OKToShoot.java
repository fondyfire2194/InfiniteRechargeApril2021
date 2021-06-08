// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.RevShooterSubsystem;

public class OKToShoot extends CommandBase {
  /** Creates a new OKToShoot. */
  RevShooterSubsystem m_shooter;

  public OKToShoot(RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.driverOKShoot = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.driverOKShoot = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
