/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StopShoot extends InstantCommand {
  private final RevShooterSubsystem shooter;
  
  private final CellTransportSubsystem transport;

  public StopShoot(RevShooterSubsystem shooter, CellTransportSubsystem transport) 
  {
    this.shooter = shooter;
    this.transport = transport;
    addRequirements(shooter, transport);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transport.runLeftBeltMotor(0.);
    transport.runFrontRollerMotor(0.);
    transport.runRearRollerMotor(0.);
    shooter.spinAtRpm(0);
  }
}
