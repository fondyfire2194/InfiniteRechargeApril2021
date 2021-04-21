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
import frc.robot.Constants.CellTransportConstants;
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
  private double speed;
  private double time;

  public ShootCells(RevShooterSubsystem shooter, CellTransportSubsystem transport, Compressor compressor, double speed,
      double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.transport = transport;
    this.compressor = compressor;
    this.speed = speed;
    this.time = time;

    addRequirements(shooter, transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    shooter.requiredSpeedLast = 0.;
    shooter.requiredSpeed = speed;
    shooter.shootTime = time;
    compressor.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.spinAtRpm(shooter.requiredSpeed);
    if (Timer.getFPGATimestamp() > startTime + 1) {
      // belts.pulseBelt(-.5, .5, .25);
      // transport.runBeltMotor(CellTransportConstants.BELT_SPEED);
      transport.runFrontRollerMotor(CellTransportConstants.FRONT_SHOOT_SPEED);
      transport.runRearRollerMotor(CellTransportConstants.REAR_SHOOT_SPEED);
      shooter.shootTimeRemaining = startTime + shooter.shootTime - Timer.getFPGATimestamp();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // transport.runLeftBeltMotor(0.);
    transport.runFrontRollerMotor(0.);
    transport.runRearRollerMotor(0.);
    shooter.spinAtRpm(0);
    compressor.start();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > startTime + shooter.shootTime;
  }
}
