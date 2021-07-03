/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LogTiTuTrack extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "AveDist", "LeftRate", "GyroYaw", "TiltAng", "TiltRate", "DegVert", "OnTgt",
      "TurretAngle", "TurretRate", "DegHor", "HOnTgt" };

  public static String[] units = { "Sec", "M", "MPS", "Degrees", "Degrees", "DPS", "Degrees", "T/F", "Degrees", "DPS",
      "Degrees", "T/F" };

  private int loopCtr;
  private boolean fileOpenNow;

  private final RevDrivetrain m_drive;
  private final RevTiltSubsystem m_tilt;
  private final RevTurretSubsystem m_turret;
  private final LimeLight m_limelight;

  private double logTime;

  private double horOnTgt;
  private double vertOnTgt;

  public LogTiTuTrack(RevDrivetrain drive, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    m_tilt = tilt;
    m_turret = turret;
    m_limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope5 = m_drive.driveLogger.init("TestRun", "Track", names, units);

    loopCtr = 0;
    fileOpenNow = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horOnTgt = 0;
    vertOnTgt = 0;

    // allow 1 second for file to be opened

    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;

    }
    // log data every shot
    if (fileOpenNow)
      m_drive.driveLogInProgress = true;
    if (logTime == 0)

      logTime = Timer.getFPGATimestamp();

    if (Timer.getFPGATimestamp() > logTime + .1) {

      if (m_limelight.getHorOnTarget(1))
        horOnTgt = 1;

      if (m_limelight.getHorOnTarget(1))
        vertOnTgt = 1;

      logTime = Timer.getFPGATimestamp();

      m_drive.driveLogger.writeData(logTime, m_drive.getAverageDistance(), m_drive.getLeftRate(), m_drive.getYaw(),
          m_tilt.getAngle(), m_tilt.getSpeed(), m_limelight.getdegVerticalToTarget(), vertOnTgt, m_turret.getAngle(),
          m_turret.getSpeed(), m_limelight.getdegRotationToTarget(), horOnTgt);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_drive.driveLogger.close();
    m_drive.endDriveFile = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drive.endDriveFile;
  }
}
