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
  private double tiltDistanceTolerance;
  private double turretDistanceTolerance;

  public ShootCells(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight limelight,
      CellTransportSubsystem transport, Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;
    m_limelight = limelight;
    m_tilt = tilt;
    m_turret = turret;
    m_time = time;
    m_shooter.startShooter = true;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();
    m_shooter.shootTime = m_time;
    m_compressor.stop();
    m_shooter.isShooting = false;
    m_transport.holdCell();
    cellsShot = 0;
    shotStartTime = 0;
    cellAvailable = false;
    m_limelight.setLEDMode(LedMode.kpipeLine);
    m_limelight.setPipeline(m_limelight.noZoomPipeline);
    m_limelight.useVision = true;
    m_shooter.logTrigger = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretDistanceTolerance = m_shooter.getTurretTolerance(m_shooter.calculatedCameraDistance);
    tiltDistanceTolerance = m_shooter.getTiltTolerance(m_shooter.calculatedCameraDistance);

    loopctr++;

    boolean inAuto = DriverStation.getInstance().isAutonomous();

    m_shooter.requiredMps = m_shooter.adjustedCameraMPS;

    m_shooter.okToShoot = (m_limelight.getVertOnTarget(tiltDistanceTolerance)
        && m_limelight.getHorOnTarget(turretDistanceTolerance)) || m_shooter.useDriverSpeed;

    if (m_shooter.okToShoot && !m_shooter.useDriverSpeed)
      m_shooter.startShooter = true;

    if (m_shooter.atSpeed() && m_shooter.okToShoot || m_shooter.isShooting) {

      m_shooter.isShooting = true;

    }

    if (m_shooter.isShooting) {
      m_transport.runFrontRollerMotor();
      m_transport.runRearRollerMotor();
      m_transport.runLeftBeltMotor(.5);
      m_transport.runRightBeltMotor(-.5);

    }

    if (m_shooter.isShooting && cellAvailable && !m_shooter.shotInProgress) {
      shotStartTime = Timer.getFPGATimestamp();
      m_shooter.shotInProgress = true;
      cellsShot++;
      cellAvailable = false;

    }

    // if (shotInProgress) {
    // m_limelight.setSnapshot(Snapshot.kon);
    // } else {
    // m_limelight.setSnapshot(Snapshot.koff);
    // }

    if (m_shooter.shotInProgress && Timer.getFPGATimestamp() > (shotStartTime + shotTime) && m_shooter.atSpeed()) {
      m_shooter.shotInProgress = false;
    }

    m_shooter.okToShoot = m_shooter.isShooting && (inAuto || !m_shooter.shootOne);

    getNextCell = m_shooter.okToShoot && !m_shooter.shotInProgress && !cellAvailable && m_shooter.atSpeed();

    if (getNextCell || cellReleased) {
      releaseOneCell();
    }

    // SmartDashboard.putBoolean("OKShoot", okToShoot);
    // SmartDashboard.putBoolean("SHIP", m_shooter.shotInProgress);
    // SmartDashboard.putBoolean("SHSTD", shootStarted);
    // SmartDashboard.putBoolean("CAvail ", cellAvailable);
    // SmartDashboard.putNumber("CLSSHT", cellsShot);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.releaseCell();
    m_transport.stopBelts();
    m_transport.stopRollers();
    m_compressor.start();
    m_shooter.shotInProgress = false;
    m_shooter.endShootFile = true;
    m_shooter.isShooting = false;
    m_shooter.logTrigger = false;
    m_shooter.setNotOKShootDriver();
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