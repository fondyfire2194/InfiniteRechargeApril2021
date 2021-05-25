// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

public class OKToShoot extends CommandBase {
  /** Creates a new OKToShoot. */
  RevShooterSubsystem m_shooter;
  RevDrivetrain m_drive;
  LimeLight m_limelight;

  public OKToShoot(RevShooterSubsystem shooter, LimeLight limelight, RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    m_shooter = shooter;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.useVision = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.lockedForVision = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.atSpeed() && m_limelight.getHorOnTarget() && m_limelight.getVertOnTarget() && m_drive.isStopped();
  }
}
