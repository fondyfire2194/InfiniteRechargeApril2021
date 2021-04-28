// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class CalculateTargetDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;

  private double baseCameraHeight = FieldConstants.BASE_CAMERA_HEIGHT;
  private double targetHeight = FieldConstants.TARGET_HEIGHT;
  private double baseCameraAngle = FieldConstants.CAMERA_BASE_ANGLE;
  private double cameraHeightSlope = FieldConstants.cameraHeightSlope;
  private double cameraAngleSlope = FieldConstants.cameraAngleSlope;

  private double calculatedCameraDistance;
  private double baseDistance = 3;
  private double m_limelightVerticalAngle;
  private double cameraAngle;

  public CalculateTargetDistance(LimeLight limelight, RevTiltSubsystem tilt, RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_tilt = tilt;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  /**
   * Target distance can be calculated from
   * 
   * tan(a1+a2) = (h2-h1) / d so d = (h2-h1) / tan(a1+a2)
   * 
   * where h1 is the camera height, h2 is the target height
   * 
   * a1 is the camera angle and a2 is the vertical angle the camera reports of the
   * target
   * 
   * The camera height varies as the tilt axis moves to lock on to the target so
   * it consists of a base height and a varying height.
   * 
   * Assume base camera angle = base tilt angle
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_limelight.getIsTargetFound()) {

      m_limelightVerticalAngle = m_limelight.getdegVerticalToTarget();

      double cameraHeight = baseCameraHeight
          + cameraHeightSlope * (m_tilt.getAngle() - HoodedShooterConstants.TILT_MIN_ANGLE);

      cameraAngle = baseCameraAngle + cameraAngleSlope * (m_tilt.getAngle() - HoodedShooterConstants.TILT_MIN_ANGLE);

      double tanAngleSum = Math.tan((Math.toRadians(m_limelightVerticalAngle + cameraAngle)));

      calculatedCameraDistance = (targetHeight - cameraHeight) / tanAngleSum;

    } else

    {

      calculatedCameraDistance = 0;

    }

    /**
     * Check distance calculated is in the acceptable range then interpolate speed
     * from array
     * 
     * Get whole number of meters and fetch speed for it and one above then
     * interpolate
     * 
     * 
     * 
     * 
     * 
     */
    if (calculatedCameraDistance >= 3 && calculatedCameraDistance <= 13) {
      // subtract base distance of 3 meters
      double distance = calculatedCameraDistance - baseDistance;

      int baseI = (int) distance;
      double base = (double) baseI;

      double rem = distance - base;

      double baseSpeed = m_shooter.speedFromCamera[baseI];
      double upperSpeed = m_shooter.speedFromCamera[baseI + 1];

      double speedRange = upperSpeed - baseSpeed;

      double speedAdder = speedRange * rem;

      m_shooter.cameraCalculatedSpeed = baseSpeed + speedAdder;

      m_shooter.useCameraSpeed = true;

    } else {
      m_shooter.cameraCalculatedSpeed = 0;
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
