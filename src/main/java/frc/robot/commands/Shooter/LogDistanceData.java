/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogDistanceData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "LeftMeters", "RightMeters", "AveMeters", "CameraDistance", "BBHeight",
      "BBWidth", "TargetArea", "AspectRatio", "RobotSpeed", "TiltAngle", "VertToTarget", "Tilt+Vert", "TurretAngle",
      "HorToTarget", "Turret+Hor", "GyroYaw" };
  public static String[] units = { "Number", "Meters", "Meters", "Meters", "Meters", "Pixels", "Pixels", "SqPixels",
      "Ratio", "MPS", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final RevDrivetrain m_drive;
  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;

  public LogDistanceData(RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      RevShooterSubsystem shooter, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_limelight = limelight;
    m_turret = turret;
    m_tilt = tilt;
    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope = m_shooter.simpleCSVLogger.init("TestRun", "Data", names, units);
    SmartDashboard.putNumber("OPE", ope);
    loopCtr = 0;
    fileOpenNow = false;
    step = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data every 2 pixels
    if (fileOpenNow)
      loopCtr++;
    if (loopCtr >= 25) {
      loopCtr = 0;
      step++;
      m_shooter.simpleCSVLogger.writeData((double) step, -m_drive.getLeftDistance(), -m_drive.getRightDistance(),
          -m_drive.getAverageDistance(), m_shooter.calculatedCameraDistance, m_limelight.getBoundingBoxHeight(),
          m_limelight.getBoundingBoxWidth(), m_limelight.getTargetArea(), m_limelight.getAspectRatio(),
          m_drive.getLeftRate(), m_tilt.getAngle(), m_limelight.getdegVerticalToTarget(),
          m_tilt.getAngle() + m_limelight.getdegVerticalToTarget(), m_turret.getAngle(),
          m_limelight.getdegRotationToTarget(), m_turret.getAngle() + m_limelight.getdegRotationToTarget(),
          m_drive.getYaw());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_shooter.simpleCSVLogger.close();
    m_shooter.endFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endFile;
  }
}
