// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.commands.RobotDrive.PickupMoveVelocity;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Trench4Macro extends SequentialCommandGroup {
  /** Creates a new Position2Macro. */
  public Trench4Macro(RevDrivetrain drive, RevShooterSubsystem shooter, RevTurretSubsystem turret,
      RevTiltSubsystem tilt, CellTransportSubsystem transport, RearIntakeSubsystem intake, LimeLight limelight) {
    // Add your commands in the a drive,ddCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetGyro(drive), new ResetEncoders(drive), new PickupMoveVelocity(drive, 5., 1.5),
        new SetShotPositionTrenchSecondBall(shooter, turret, tilt, transport, intake, limelight));
  }
}