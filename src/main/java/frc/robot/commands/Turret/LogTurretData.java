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
  public final String[] names = { "Step", "ProgRunning", "UseVision", "ValidTarget", "TargetAngle", "TurretAngle",
      "Offset", "LockPE", "DegVertToTgt", "CorrEndPt" };

  public static String[] units = { "Number", "1Hold2Pos3Vis", "T/F", "T/F", "Degrees", "Degrees", "PU", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;
  private double logTime;

  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;

  private double validTargetSeen;
  private double useVision;

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

      if (m_limelight.useVision)
        useVision = 1;
      else
        useVision = 0;

      if (m_turret.validTargetSeen)
        validTargetSeen = 1;
      else
        validTargetSeen = 0;

      m_turret.turretLogger.writeData(logTime, m_turret.programRunning,

          useVision, validTargetSeen, m_turret.targetAngle, m_turret.getAngle(), m_turret.targetHorizontalOffset,

          m_turret.m_turretLockController.getPositionError(), m_limelight.getdegVerticalToTarget(),
          m_turret.correctedEndpoint);
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
