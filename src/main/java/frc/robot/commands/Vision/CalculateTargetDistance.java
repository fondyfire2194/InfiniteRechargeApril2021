// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class CalculateTargetDistance extends CommandBase {
  /** Creates a new CalculateTargetDistance. */
  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private final RevTurretSubsystem m_turret;

  private double baseCameraHeight = FieldConstants.BASE_CAMERA_HEIGHT;
  private double maxCameraHeight = FieldConstants.MAX_CAMERA_HEIGHT;
  private double targetHeight = FieldConstants.PORT_CENTER_HEIGHT;
  private double heightDifference;

  private double m_cameraVerticalError;
  private double cameraAngle;
  private double cameraHeightSlope = (maxCameraHeight - baseCameraHeight) / 30;// inches per tilt degree

  public CalculateTargetDistance(LimeLight limelight, RevTiltSubsystem tilt, RevTurretSubsystem turret,
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
   * As the tilt angle to the ground increases, the camera angle to the target
   * falls.
   * 
   * Low tilt angles mean high camera angles!
   * 
   * Limelight must have the Y offset to the center of the port - if not it
   * provides the cener of the target
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
   * = tan-1(h2-h1)/d with tilt angle being camera angle
   * 
   * 
   * 
   */

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // baseCameraHeight= Pref.getPref("LimelightHeight");

    // maxCameraHeight = Pref.getPref("LimelightMaxHeight");

    cameraHeightSlope = (maxCameraHeight - baseCameraHeight) / 30;

    double cameraHeight = baseCameraHeight + (cameraHeightSlope * m_tilt.getMotorDegrees());

    heightDifference = targetHeight - cameraHeight;

    if (m_limelight.useVision && (m_limelight.getIsTargetFound() || RobotBase.isSimulation())) {

      m_cameraVerticalError = m_limelight.getdegVerticalToTarget() - m_limelight.verticalOffset;

      if (RobotBase.isSimulation()) {
        m_cameraVerticalError = 0;

      }
      // tilt counts up camera angle counts down
      cameraAngle = m_tilt.getCameraAngle();

      double tanAngleSum = Math.tan((Math.toRadians(m_cameraVerticalError + cameraAngle)));

      m_shooter.calculatedCameraDistance = (heightDifference) / tanAngleSum;

      m_tilt.driverAdjustAngle = Math.toDegrees(Math.atan(m_tilt.adjustMeters / m_shooter.calculatedCameraDistance));

      m_turret.driverAdjustAngle = Math
          .toDegrees(Math.atan(m_turret.adjustMeters / m_shooter.calculatedCameraDistance));

    } else

    {

      m_shooter.calculatedCameraDistance = -1;
      m_tilt.driverAdjustAngle = 0.1;
      m_turret.driverAdjustAngle = 0.1;

    }
    m_shooter.shotDistance = Math.sqrt((m_shooter.calculatedCameraDistance * m_shooter.calculatedCameraDistance)
        + (FieldConstants.SHOT_HEIGHT_SQUARED));

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
