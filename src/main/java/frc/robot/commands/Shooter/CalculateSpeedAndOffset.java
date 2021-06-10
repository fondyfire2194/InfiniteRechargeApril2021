// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class CalculateSpeedAndOffset extends CommandBase {
  /** Creates a new CalculateSpeedAndOffset. */
  private RevTiltSubsystem m_tilt;
  private LimeLight m_limelight;
  private RevShooterSubsystem m_shooter;

  public CalculateSpeedAndOffset(RevShooterSubsystem shooter, RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_limelight = limelight;
    m_tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] temp = new double[] { 0, 0 };

    if (!DriverStation.getInstance().isAutonomous()) {
      if (!m_shooter.useSetupSlider) {
        if (Math.abs(m_limelight.getdegVerticalToTarget()) < .5)
          temp = m_shooter.calculateMPSandYOffset(
              (m_tilt.getCameraAngle()) + m_limelight.getdegVerticalToTarget() - m_tilt.targetVerticalOffset);
      } else {
        m_shooter.requiredMps = m_shooter.shooterSpeed.getDouble(2);
        m_tilt.targetVerticalOffset = m_shooter.setupVertOffset.getDouble(0);
      }
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
