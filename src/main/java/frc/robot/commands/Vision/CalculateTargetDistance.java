// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
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
  private double heightDifference;

  private double m_limelightVerticalAngle;
  private double cameraAngle;
  private double tiltAngle;

  public CalculateTargetDistance(LimeLight limelight, RevTiltSubsystem tilt, RevShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_tilt = tilt;
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    heightDifference = targetHeight - baseCameraHeight;
  }

  /**
   * As the tilt angle to the ground increases, the camera angle to the target
   * falls.
   * 
   * Low tilt angles mean high camera angles.Camera angle = 90 - tilt angle.
   * 
   * Minimum tilt angle measured to the ground is 59 degrees. Max is 89 degrees
   * 
   * So min camera to target angle is 1 degree and max is 31 degrees
   * 
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
   * When locked on target a2 is 0 so d = (h2-h1)/tan(cameraAngle) and cameraAngle
   * = tan-1(h2-h1)/d with tilt angle being 90 -camera angle
   * 
   * Base camera angle = 90 - base tilt angle
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_limelight.getIsTargetFound() || RobotBase.isSimulation()) {

      m_limelightVerticalAngle = m_limelight.getdegVerticalToTarget();

      tiltAngle = m_tilt.getAngle();

      if (RobotBase.isSimulation()) {
        m_limelightVerticalAngle = 0;

      }

      cameraAngle = 90 - tiltAngle;

      double tanAngleSum = Math.tan((Math.toRadians(m_limelightVerticalAngle + cameraAngle)));

      m_shooter.calculatedCameraDistance = (heightDifference) / tanAngleSum;

    } else

    {

      m_shooter.calculatedCameraDistance = 0;

    }

    m_shooter.calculateSpeedFromDistance(m_shooter.calculatedCameraDistance);
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
