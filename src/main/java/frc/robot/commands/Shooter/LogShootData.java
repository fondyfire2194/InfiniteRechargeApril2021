/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class LogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "ActSpeed", "AtSpeed", "IsShooting", "LeftCurrent", "CellReleasing",
      "ArmPosn" };

  public static String[] units = { "Seconds", "MPS", "T/F", "T/F", "Amps", "T/F", "T/F", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;

  private final RevShooterSubsystem m_shooter;
  private final CellTransportSubsystem m_transport;

  private double isShooting;

  private double shooterAtSpeed;
  private double servoArmReleasing;
  private double logTime;

  public LogShootData(RevShooterSubsystem shooter, CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_transport = transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope1 = m_shooter.shootLogger.init("TestRun", "Shoot", names, units);
    SmartDashboard.putNumber("OPE1", ope1);
    loopCtr = 0;
    fileOpenNow = false;
    logTime = 0;

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
    // log data every 100ms
    if (fileOpenNow)
      m_shooter.shootLogInProgress = true;
    if (logTime == 0)
      logTime = Timer.getFPGATimestamp();

    if (Timer.getFPGATimestamp() > logTime + .1) {
      logTime = Timer.getFPGATimestamp();

      if (m_shooter.atSpeed())
        shooterAtSpeed = 1;
      else
        shooterAtSpeed = 0;

      if (m_shooter.shotInProgress)
        servoArmReleasing = 1;
      else
        servoArmReleasing = 0;

      m_shooter.shootLogger.writeData(logTime, m_shooter.getMPS(), shooterAtSpeed, isShooting, m_shooter.getLeftAmps(),
          servoArmReleasing, (double) m_transport.getArmAngle());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_shooter.shootLogger.close();
    m_shooter.endShootFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endShootFile;
  }
}
