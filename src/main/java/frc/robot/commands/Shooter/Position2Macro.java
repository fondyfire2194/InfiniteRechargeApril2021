// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Position2Macro extends SequentialCommandGroup {
  /** Creates a new Position2Macro. */
  public Position2Macro(RevDrivetrain drive, RevShooterSubsystem shooter, RevTurretSubsystem turret,
      RevTiltSubsystem tilt, CellTransportSubsystem transport, LimeLight limelight) {
    // Add your commands in the a drive,ddCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyro(drive), new ResetEncoders(drive), new PickupMove(drive, 2.5, .4, .5),
        new SetShotPosition2(shooter, turret, tilt, transport, limelight));
  }
}
