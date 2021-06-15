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
import frc.robot.LimelightControlMode.Snapshot;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootCells extends CommandBase {
  /**
   * Creates a new ShootCells
   * 
   */
  private final RevShooterSubsystem m_shooter;
  private final CellTransportSubsystem m_transport;
  private final Compressor m_compressor;
  private final LimeLight m_limelight;
  private double m_time;
  private double shotTime = 1;
  private double shotStartTime;

  private boolean okToShoot;

  private boolean shootStarted;
  private boolean temp;
  private int cellsShot;
  private boolean shotInProgress;
  private double startTime;
  private boolean getNextCell;
  private boolean cellAvailable;
  private int loopCtr;
  private boolean cellReleased;
  private double cellReleasedStartTime;

  public ShootCells(RevShooterSubsystem shooter, CellTransportSubsystem transport, LimeLight limelight,
      Compressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;

    m_limelight = limelight;
    m_time = 5;

    addRequirements(m_transport);
  }

  public ShootCells(RevShooterSubsystem shooter, LimeLight limelight, CellTransportSubsystem transport,
      Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;
    m_limelight = limelight;
    temp = m_limelight.useVision;
    m_time = time;

    // addRequirements(shooter, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();
    m_shooter.shootTime = m_time;
    m_compressor.stop();
    shootStarted = false;
    m_limelight.useVision = false;
    m_transport.holdCell();
    cellsShot = 0;
    shotStartTime = 0;
    cellAvailable = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.runShooter();
    boolean inAuto = DriverStation.getInstance().isAutonomous();

    m_limelight.useVision = !m_shooter.driverOKShoot;

    if ((m_shooter.atSpeed() && m_limelight.getHorOnTarget() && m_limelight.getVertOnTarget())
        || (m_shooter.driverOKShoot && m_shooter.atSpeed()) || shootStarted == true) {

      shootStarted = true;

    }

    if (shootStarted) {
      m_transport.runFrontRollerMotor();
      m_transport.runRearRollerMotor();
      m_transport.pulseLeftBelt(.5, .2, .2);
      m_transport.pulseRightBelt(.23, .25, .5);
    }

    if (shootStarted && cellAvailable && !shotInProgress) {
      shotStartTime = Timer.getFPGATimestamp();
      shotInProgress = true;
      cellsShot++;
      cellAvailable = false;

    }

    m_shooter.logTrigger = shotInProgress;

    if (shotInProgress) {
      m_limelight.setSnapshot(Snapshot.kon);
    } else {
      m_limelight.setSnapshot(Snapshot.koff);
    }

    if (shotInProgress && Timer.getFPGATimestamp() > (shotStartTime + shotTime)) {
      shotInProgress = false;
    }

    okToShoot = shootStarted && (inAuto || !m_shooter.shootOne);

    getNextCell = okToShoot && !shotInProgress && !cellAvailable;

    if (getNextCell || loopCtr != 0) {
      releaseACell();

    }

    SmartDashboard.putBoolean("OKShoot", okToShoot);
    SmartDashboard.putBoolean("SHIP", shotInProgress);
    SmartDashboard.putBoolean("SHSTD", shootStarted);
    SmartDashboard.putBoolean("CAvail ", cellAvailable);
    SmartDashboard.putNumber("CLSSHT", cellsShot);
    // SmartDashboard.putNumber("LPCTR", loopCtr);
    // SmartDashboard.putBoolean("GNXC ", getNextCell);

  }

  // m_shooter.shootTimeRemaining=startTime+m_shooter.shootTime-Timer.getFPGATimestamp();

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.releaseCell();
    m_transport.stopBelts();
    m_transport.stopRollers();
    m_limelight.useVision = temp;
    m_compressor.start();
    shotInProgress = false;
    m_shooter.endShootFile = true;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void releaseACell() {
    loopCtr++;
    if (loopCtr < 2)
      m_transport.releaseCell();
    if (loopCtr > 50 && loopCtr < 52)
      m_transport.holdCell();
    if (loopCtr > 100) {
      cellAvailable = true;
      loopCtr = 0;
    }

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