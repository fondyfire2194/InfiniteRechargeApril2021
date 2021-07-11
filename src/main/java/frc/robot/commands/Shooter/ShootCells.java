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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class ShootCells extends CommandBase {
  /**
   * Creates a new ShootCells
   * 
   */
  private final RevShooterSubsystem m_shooter;
  private final RevTiltSubsystem m_tilt;
  private final RevTurretSubsystem m_turret;
  private final CellTransportSubsystem m_transport;
  private final RevDrivetrain m_drive;
  private final Compressor m_compressor;
  private final LimeLight m_limelight;
  private double m_time;
  private double shotTime = .7;
  private double shotStartTime;

  private boolean okToShoot;

  private boolean getNextCell;

  private boolean cellReleased;
  private double cellReleasedStartTime;

  private boolean inAuto;
  private boolean useSensors = false;
  public boolean robotStoppedFor1Sec;

  public ShootCells(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight limelight,
      CellTransportSubsystem transport, RevDrivetrain drive, Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;
    m_limelight = limelight;
    m_drive = drive;
    m_tilt = tilt;
    m_turret = turret;
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shootCellsRunning = 1.;
    m_shooter.logShooterItems = true;
    m_shooter.shootTime = m_time;
    m_compressor.stop();
    m_shooter.isShooting = false;
    m_transport.holdCell();
    m_transport.holdLeftChannel();
    m_transport.cellsShot = 0;
    shotStartTime = 0;

    m_limelight.setLEDMode(LedMode.kpipeLine);

    m_limelight.useVision = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robotStoppedFor1Sec = m_drive.robotStoppedForOneSecond;

    inAuto = DriverStation.getInstance().isAutonomous();

    okToShoot = (m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance)
        && m_limelight.getHorOnTarget(m_turret.turretVisionTolerance)) || m_shooter.useDriverSpeed;

    if (m_shooter.atSpeed() && m_transport.rollersAtSpeed && okToShoot && robotStoppedFor1Sec || m_shooter.isShooting) {

      m_shooter.isShooting = true;

    }

    if (m_shooter.isShooting && m_transport.cellAvailable && !m_shooter.shotInProgress) {
      shotStartTime = Timer.getFPGATimestamp();
      m_shooter.shotInProgress = true;
      m_transport.cellsShot++;
      m_transport.cellAvailable = false;

    }

    if (m_shooter.shotInProgress && Timer.getFPGATimestamp() > shotStartTime + shotTime) {

      m_shooter.shotInProgress = false;

    }

    m_shooter.okToShoot = m_shooter.isShooting && (inAuto || !m_shooter.shootOne);

    getNextCell = m_shooter.okToShoot && !m_shooter.shotInProgress && !m_transport.cellAvailable
        && m_transport.rollersAtSpeed && m_transport.getBallAtShoot() && m_shooter.atSpeed();

    if (getNextCell || cellReleased) {
      releaseOneCell();
    }

    if (useSensors && !m_transport.noBallatShooterForOneSecond && m_transport.getBallAtLeft()) {
      m_transport.releaseLeftChannel();
    }

    if (inAuto && m_transport.cellsShot > 2)
      m_transport.releaseLeftChannel();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.holdCell();
    m_shooter.logShooterItems = false;
    m_transport.holdLeftChannel();

    m_transport.stopRollers();
    m_compressor.start();
    m_shooter.shotInProgress = false;
    m_shooter.endShootFile = true;
    m_shooter.isShooting = false;
    m_shooter.setNotOKShootDriver();
    m_shooter.shootCellsRunning = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_transport.cellsShot > 5 || inAuto && m_transport.cellsShot >= 4;
  }

  public void releaseOneCell() {
    if (!cellReleased) {
      m_transport.releaseCell();
      cellReleased = true;
      cellReleasedStartTime = Timer.getFPGATimestamp();
    }

    if (cellReleased && Timer.getFPGATimestamp() > cellReleasedStartTime + m_transport.cellPassTime) {

      m_transport.holdCell();
    }

    if (cellReleased && Timer.getFPGATimestamp() > cellReleasedStartTime + m_transport.cellPassTime) {

      cellReleasedStartTime = 0;

      m_transport.cellAvailable = true;

      cellReleased = false;
    }

  }
}