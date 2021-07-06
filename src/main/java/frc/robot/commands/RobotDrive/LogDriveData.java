/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class LogDriveData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "LeftDist", "LeftRate", "LeftAmps", "LeftOut", "RightDist", "RightRate",
      "RightAmps", "RightOut", "GyroYaw" };

  public static String[] units = { "Sec", "M", "MPS", "Amps", "PU", "M", "MPS", "Amps", "PU", "Deg" };
  private int loopCtr;
  private boolean fileOpenNow;

  private final RevDrivetrain m_drive;

  private double logTime;

  public LogDriveData(RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope5 = m_drive.driveLogger.init("TestRun", "Drive", names, units);

    loopCtr = 0;
    fileOpenNow = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
      m_drive.logDriveItems=true;
    if (m_drive.logDriveItems && Timer.getFPGATimestamp() > logTime + .1) {

      logTime = Timer.getFPGATimestamp();

      m_drive.driveLogger.writeData(logTime, m_drive.getLeftDistance(), m_drive.getLeftRate(), m_drive.getLeftAmps(),
          m_drive.getLeftOut(),

          m_drive.getRightDistance(), m_drive.getRightRate(), m_drive.getRightAmps(), m_drive.getRightOut(),
          m_drive.getYaw());
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
