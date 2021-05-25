// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWhenReady extends SequentialCommandGroup {
  /**
   * Creates a new ShootWhenReady.
   * 
   * Moves the tilt and turret to position where it can pick up the target when
   * the anticipated shoot position is reached. Sets the shoot speed to the
   * anticipated level for that position.
   * 
   */
  public ShootWhenReady(RevShooterSubsystem shooter, LimeLight limelight, RevTiltSubsystem tilt,
      RevTurretSubsystem turret, RevDrivetrain drive, CellTransportSubsystem transport, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new OKToShoot(shooter, limelight, drive), new ShootCells(shooter, transport, compressor, 5),
        new ReturnTiltTurret(turret, 0, tilt, 20, limelight, false, shooter, 1000));
  }
}