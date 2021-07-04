/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class ShootInMotion extends CommandBase {
  /**
   * Creates a new ShootCells
   * 
   */
  private final RevShooterSubsystem m_shooter;
  private final RevTiltSubsystem m_tilt;
  private final RevTurretSubsystem m_turret;
  private final RevDrivetrain m_drive;
  private final CellTransportSubsystem m_transport;
  private final Compressor m_compressor;
  private final LimeLight m_limelight;
  private double m_time;
  private double shotTime = 1;
  private double shotStartTime;

  private boolean okToShoot;

  private int cellsShot;
  private double startTime;
  private boolean getNextCell;
  private boolean cellAvailable;
  private boolean cellReleased;
  private double cellReleasedStartTime;

  private int loopctr;

  public static double startTiltAngle = 24;
  public static double startTurretAngle = -36;
  public static double startShootSpeed = 30;
  public static double startTiltOffset;
  public static double startTurretOffset;

  public static double endTiltAngle = 13;
  public static double endTurretAngle = -16;
  public static double endShootSpeed = 40;
  public static double endTiltOffset = 6;
  public static double endTurretOffset = 0;

  public static double distance = 4;

  private double turretOffsetChangePerMeter = (endTurretOffset - startTurretOffset) / distance;
  private double tiltOffsetChangePerMeter = (endTiltOffset - startTiltOffset) / distance;
  private double shootMPSChangePerMeter = (endShootSpeed - startShootSpeed) / distance;

  public ShootInMotion(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight limelight, CellTransportSubsystem transport, RevDrivetrain drive, Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_drive = drive;
    m_compressor = compressor;
    m_limelight = limelight;
    m_tilt = tilt;
    m_turret = turret;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();
    m_shooter.shootTime = m_time;
    m_shooter.shotInProgress = false;
    m_compressor.stop();
    m_shooter.isShooting = false;
    m_transport.holdCell();
    cellsShot = 0;
    shotStartTime = 0;
    cellAvailable = false;
    m_limelight.setLEDMode(LedMode.kpipeLine);
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_limelight.useVision = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    loopctr++;

    okToShoot = (m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance)
        && m_limelight.getHorOnTarget(m_turret.turretVisionTolerance)) && m_shooter.atSpeed();

    m_shooter.isShooting = okToShoot;

    getNextCell = okToShoot && !m_shooter.shotInProgress && !cellAvailable;

    if (getNextCell || cellReleased) {
      releaseOneCell();
    }

    if (okToShoot && cellAvailable && !m_shooter.shotInProgress) {
      shotStartTime = Timer.getFPGATimestamp();
      m_shooter.shotInProgress = true;
      cellsShot++;
      cellAvailable = false;

    }

    if (m_shooter.shotInProgress && Timer.getFPGATimestamp() > (shotStartTime + shotTime)) {
      m_shooter.shotInProgress = false;
    }

    SmartDashboard.putBoolean("OKShoot", okToShoot);
    SmartDashboard.putBoolean("SHIP", m_shooter.shotInProgress);

    SmartDashboard.putBoolean("CAvail ", cellAvailable);
    SmartDashboard.putNumber("CLSSHT", cellsShot);

    distance = -m_drive.getAverageDistance();

    m_tilt.tiltOffsetChange = distance * tiltOffsetChangePerMeter;
    m_turret.turretOffsetChange = distance * turretOffsetChangePerMeter;
    m_shooter.shooterFPSChange = distance * shootMPSChangePerMeter;

    m_tilt.targetVerticalOffset += m_tilt.tiltOffsetChange;
    m_turret.targetHorizontalOffset += m_turret.turretOffsetChange;

    m_limelight.setHorizontalOffset(m_tilt.targetVerticalOffset);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.holdCell();
    m_compressor.start();
    m_shooter.shotInProgress = false;
    m_shooter.endShootFile = true;
    m_shooter.isShooting = false;

    m_shooter.shooterFPSChange = 0;

    m_tilt.targetVerticalOffset -= m_tilt.tiltOffsetChange;
    m_turret.targetHorizontalOffset -= m_turret.turretOffsetChange;
    m_tilt.tiltOffsetChange = 0;
    m_turret.turretOffsetChange = 0;

    m_transport.stopBelts();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cellsShot > (int) m_time;
  }

  public void releaseOneCell() {
    if (!cellReleased) {
      m_transport.releaseCell();
      cellReleased = true;
      cellReleasedStartTime = Timer.getFPGATimestamp();
    }
    if (cellReleased && Timer.getFPGATimestamp() > cellReleasedStartTime + m_transport.cellPassTime) {
      m_transport.holdCell();
      cellReleasedStartTime = 0;

      cellAvailable = true;
      cellReleased = false;
    }

  }
}