/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootOne extends CommandBase {
  /**
   * Creates a new ShootCells
   * 
   */
  private final RevShooterSubsystem m_shooter;
  private final CellTransportSubsystem m_transport;
  private final Compressor m_compressor;

 

  public ShootOne(RevShooterSubsystem shooter, CellTransportSubsystem transport, 
      Compressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_transport = transport;
    m_compressor = compressor;


    addRequirements(m_transport);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_compressor.stop();
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.startShooter = true;
    m_transport.runFrontRollerMotor();
    m_transport.runRearRollerMotor();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.moveCellArm(m_transport.cellArmHoldCell);
    m_transport.stopBelts();
    m_transport.stopRollers();
    m_shooter.stop();

    m_compressor.start();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
