/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogTiltData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "ProgRunning", "UseVision", "ValidTarget", "TargetAngle", "TiltAngle",
      "Offset", "LockPE", "DegHorToTgt", "CorrEndPt", "Out", "Speed" };

  public static String[] units = { "Sec", "1Hold2Pos3Vis", "T/F", "T/F", "Degrees",  "Degrees", "Degrees",
      "PU", "Degrees", "PU", "MPSec" };
  private int loopCtr;
  private boolean fileOpenNow;

  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;

  private double validTargetSeen;
  private double useVision;
  private double logTime;

  public LogTiltData(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_limelight = limelight;
    m_tilt = tilt;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope4 = m_tilt.tiltLogger.init("TestRun", "Tilt", names, units);

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
      m_tilt.tiltLogInProgress = true;
    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (m_tilt.logTiltItems && Timer.getFPGATimestamp() > logTime + .1) {

      logTime = Timer.getFPGATimestamp();

      if (m_limelight.useVision)
        useVision = 1;
      else
        useVision = 0;

      if (m_tilt.validTargetSeen)
        validTargetSeen = 1;
      else
        validTargetSeen = 0;

      m_tilt.tiltLogger.writeData(logTime, m_tilt.programRunning,

          useVision, validTargetSeen, m_tilt.targetAngle, m_tilt.getAngle(), m_tilt.targetVerticalOffset,

           m_tilt.tiltLockController.getPositionError(),

          m_limelight.getdegVerticalToTarget(), m_tilt.correctedEndpoint, m_tilt.getOut(), m_tilt.getSpeed());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_tilt.tiltLogger.close();
    m_tilt.endTiltFile = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tilt.endTiltFile;
  }
}
