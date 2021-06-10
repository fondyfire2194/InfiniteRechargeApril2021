/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
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
  private double shooterAccTime = 1;
  private boolean shootStarted;
  private boolean temp;
  private double temp1;
  private double position1 = 0;
  private double position2 = .2;
  private int loopCtr;

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

    addRequirements(shooter, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_shooter.requiredMpsLast = 0.;
    m_shooter.shootTime = m_time;
    m_compressor.stop();
    shootStarted = false;
    m_limelight.useVision = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.startShooter = true;

    if ((m_shooter.atSpeed() && m_limelight.getHorOnTarget() && m_limelight.getVertOnTarget())
        || m_shooter.driverOKShoot || shootStarted == true) {

      shootStarted = true;

      if (loopCtr == 0)
        temp1 = position1;
      m_transport.moveCellArm(temp1);
      loopCtr++;
      if (loopCtr > 5)
        temp1 = position2;
      if (loopCtr > 10)
        loopCtr = 0;
    }

    m_transport.runFrontRollerMotor();
    m_transport.runRearRollerMotor();

    m_shooter.shootTimeRemaining = startTime + m_shooter.shootTime - Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.moveCellArm(position1);
    m_transport.stopBelts();
    m_transport.stopRollers();
    m_shooter.stop();
    m_limelight.useVision = temp;
    m_compressor.start();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > startTime + m_shooter.shootTime) && m_shooter.shootTime != 0;
  }
}
