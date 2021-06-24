/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogTiltData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "TargetAngle", "TiltAngle", "PositionError", "TargetSeen", "ValidTarget",
      "DegVerToTgt", "VisErDiff", "CorrEndPt" };
  public static String[] units = { "Number", "Degrees", "Degrees", "Degrees", "OnOff", "OnOff", "Degrees", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;
  private int step;

  private final LimeLight m_limelight;
  private final RevTiltSubsystem m_tilt;

  private double validTargetSeen;
  private double targetSeen;

  public LogTiltData(RevTiltSubsystem tilt, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_limelight = limelight;
    m_tilt = tilt;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope4 = m_tilt.tiltLogger.init("TestRun", "Tilt", names, units);
    SmartDashboard.putNumber("OPE4", ope4);
    loopCtr = 0;
    fileOpenNow = false;
    step = 0;

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
      loopCtr++;
    
    if (m_tilt.logTrigger && loopCtr >= 5) {
      loopCtr = 0;
      step++;

      if (m_limelight.getIsTargetFound())
        targetSeen = 1;
      else
        targetSeen = 0;

      if (m_tilt.validTargetSeen)
        validTargetSeen = 1;
      else
        validTargetSeen = 0;

      m_tilt.tiltLogger.writeData((double) step,

          m_tilt.targetAngle, m_tilt.getAngle(), m_tilt.positionError, targetSeen, validTargetSeen,
          m_limelight.getdegVerticalToTarget(), m_tilt.visionErrorDifference, m_tilt.correctedEndpoint);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_tilt.tiltLogger.close();
    m_tilt.endTiltFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tilt.endTiltFile;
  }
}
