/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShootCells extends CommandBase {
  /**
   * Creates a new ShootCells.
   * 
   */
  private final RevShooterSubsystem shooter;
  private final CellTransportSubsystem transport;
  private final Compressor compressor;
  private double startTime;
  private double time;
  private final double rollerPctofShooter = .75;

  public ShootCells(RevShooterSubsystem shooter, CellTransportSubsystem transport, Compressor compressor, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.transport = transport;
    this.compressor = compressor;
    this.time = time;

    addRequirements(shooter, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shooter.requiredMpsLast = 0.;
    shooter.shootTime = time;
    compressor.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.runShooter();
    
    if (Timer.getFPGATimestamp() > startTime + 1) {
      double shooterOut = shooter.getLeftPctOut();
      double rollerMotorOut = shooterOut * rollerPctofShooter;
      transport.runFrontRollerMotor(rollerMotorOut);
      transport.runRearRollerMotor(-rollerMotorOut);
      shooter.shootTimeRemaining = startTime + shooter.shootTime - Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // transport.runLeftBeltMotor(0.);
    transport.runFrontRollerMotor(0.);
    transport.runRearRollerMotor(0.);
    shooter.stop();
    compressor.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() > startTime + shooter.shootTime) && shooter.shootTime != 0;
  }
}
