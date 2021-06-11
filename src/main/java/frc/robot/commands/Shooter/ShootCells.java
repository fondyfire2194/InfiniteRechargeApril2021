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
  private double startTime;
  private double m_time;
  private double shootOneTime;
  private double cellReleaseStartTime;

  private boolean shootStarted;
  private boolean temp;
  private int cellsShot;

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
    startTime = 0;
    m_time = time;

    addRequirements(shooter, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_shooter.requiredMpsLast = 0.;
    m_shooter.shootTime = m_time;
    shootOneTime = 0;
    m_compressor.stop();
    shootStarted = false;
    m_shooter.shootOne = true;
    m_limelight.useVision = false;
    m_transport.holdCell();
    cellsShot = 1;
    cellReleaseStartTime = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((m_shooter.atSpeed() && m_limelight.getHorOnTarget() && m_limelight.getVertOnTarget())
        || m_shooter.driverOKShoot || shootStarted == true) {

      shootStarted = true;
    }
    if (m_shooter.startShooter = true) {
      m_transport.runFrontRollerMotor();
      m_transport.runRearRollerMotor();
    }

    boolean inAuto = DriverStation.getInstance().isAutonomous();

    if ((inAuto || !m_shooter.shootOne) && shootStarted && shootOneTime == 0) {
      shootOneTime = Timer.getFPGATimestamp();
    }

    if ((inAuto || !m_shooter.shootOne) && shootOneTime != 0
        && Timer.getFPGATimestamp() > (shootOneTime + m_shooter.shooterRecoverTime) && m_shooter.atSpeed()) {
      m_transport.releaseCell();
      cellReleaseStartTime = Timer.getFPGATimestamp();
    }
    if (shootOneTime != 0 && Timer.getFPGATimestamp() > (cellReleaseStartTime + m_transport.cellReleasedTime)) {
      m_transport.holdCell();
      shootOneTime = 0;
      cellsShot++;
      cellReleaseStartTime = 0;
    }

    m_shooter.shootTimeRemaining = startTime + m_shooter.shootTime - Timer.getFPGATimestamp();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.releaseCell();
    m_transport.stopBelts();
    m_transport.stopRollers();
    m_shooter.stop();
    m_limelight.useVision = temp;
    m_compressor.start();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cellsShot > 7;
  }

}
