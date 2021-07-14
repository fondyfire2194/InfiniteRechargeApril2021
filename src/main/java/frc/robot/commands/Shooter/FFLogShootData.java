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
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

public class FFLogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Time", "ElTime", "ShCelRng", "ActSpeed", "Amps",

      "AtSpeed", "OkToShoot", "isShooting", "ShotInProgress", "Cell Available",

      "BallAtLeft", "BallAtShooter", "RobotStopped",

      "ArmAngle", "LeftAngle", "Yaw" };

  public static String[] units = { "Seconds", "Seconds", "MPS", "Amps",

      "T/F", "T/F",

      "T/F", "T/F", "T/F", "T/F", "T/F",

      "Degrees", "Degrees", "Degrees" };

  private int loopCtr;
  private boolean fileOpenNow;

  private final RevShooterSubsystem m_shooter;
  private final CellTransportSubsystem m_transport;
  private final RevDrivetrain m_drive;

  private double logTime;
  private double firstLogTime;

  public FFLogShootData(RevShooterSubsystem shooter, CellTransportSubsystem transport, RevDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = shooter;
    m_transport = transport;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope1 = m_shooter.shootLogger.init("TestRun", "Shoot", names, units);
    SmartDashboard.putNumber("OPE1", ope1);
    loopCtr = 0;
    fileOpenNow = false;
    logTime = 0;
    firstLogTime=0;

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
m_shooter.logShooterItems=true;
    if (m_shooter.logShooterItems && Timer.getFPGATimestamp() > logTime + .1) {
      logTime = Timer.getFPGATimestamp();

      if (firstLogTime == 0)
        firstLogTime = logTime;
        SmartDashboard.putNumber("FTL", firstLogTime);

      m_shooter.shootLogger.writeData(logTime, logTime - firstLogTime, m_shooter.shootCellsRunning, m_shooter.getMPS(),
          m_shooter.getLeftAmps(), m_shooter.atSpeed() ? 1.0 : 0.0, m_shooter.okToShoot ? 1.0 : 0.0,
          m_shooter.isShooting ? 1.0 : 0.0, m_shooter.shotInProgress ? 1.0 : 0.0, m_transport.cellAvailable ? 1.0 : 0.0,
          m_transport.getBallAtLeft() ? 1.0 : 0.0, m_transport.getBallAtShoot() ? 1.0 : 0.0,
          m_drive.robotStoppedForOneSecond ? 1.0 : 0.0, (double) m_transport.getArmAngle(),
          (double) m_transport.getLeftAngle(), m_drive.getYaw());
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
