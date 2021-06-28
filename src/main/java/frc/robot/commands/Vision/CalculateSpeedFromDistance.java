// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CalculateSpeedFromDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private final RevTurretSubsystem m_turret;
  private LinearFilter m_filter = LinearFilter.movingAverage(5);
  private double tempSpeed;

  private double baseCameraHeight = FieldConstants.BASE_CAMERA_HEIGHT;
  private double maxCameraHeight = FieldConstants.MAX_CAMERA_HEIGHT;

  public CalculateSpeedFromDistance(LimeLight limelight, RevTiltSubsystem tilt, RevTurretSubsystem turret,
      RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_tilt = tilt;
    m_shooter = shooter;
    m_turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  /**
   * Calculates the final MPS for the shooter from the camera distance and camera
   * vertical error
   * 
   * 
   * 
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double baseSpeed = m_shooter.calculateMPSFromDistance(m_shooter.calculatedCameraDistance);

    double speedChangeFromCameraVerticalError = m_shooter
        .calculateSpeedChangeFromCameraVerticalError(m_limelight.getdegVerticalToTarget(), baseSpeed);
    tempSpeed = baseSpeed + speedChangeFromCameraVerticalError;

    m_shooter.calculatedCameraDistance = m_filter.calculate(tempSpeed);

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
