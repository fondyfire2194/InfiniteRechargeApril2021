/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTurretSubsystem;

public class LogTurretData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "ElTime", "ProgRunning", "UseVision", "ValidTarget", "TargetAngle",
      "TurretAngle", "Tolerance", "LockPE", "DegVertToTgt", "CorrEndPt", "Out", "Speed" };

  public static String[] units = { "Secs", "Secs", "1Hold2Pos3Vis", "T/F", "T/F", "Degrees", "Degrees", "PU", "Degrees",
      "PU", "MPS" };

  private int loopCtr;
  private boolean fileOpenNow;
  private double logTime;
  private double firstLogTime;

  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;

  public LogTurretData(RevTurretSubsystem turret, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_limelight = limelight;
    m_turret = turret;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope4 = m_turret.turretLogger.init("TestRun", "Turret", names, units);

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
      m_turret.turretLogInProgress = true;
    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (m_turret.logTurretItems && Timer.getFPGATimestamp() > logTime + .1) {
      logTime = Timer.getFPGATimestamp();

      if (firstLogTime == 0)
        firstLogTime = logTime;

      m_turret.turretLogger.writeData(logTime, logTime - firstLogTime, m_turret.programRunning,

          m_limelight.useVision ? 1.0 : 0.0, m_turret.validTargetSeen ? 1.0 : 0.0, m_turret.targetAngle,
          
          m_turret.getAngle(), m_turret.targetHorizontalOffset,

          m_turret.m_turretLockController.getPositionError(), m_limelight.getdegVerticalToTarget(),

          m_turret.correctedEndpoint, m_turret.getOut(), m_turret.getSpeed());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_turret.turretLogger.close();
    m_turret.endTurretFile = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.endTurretFile;
  }
}
