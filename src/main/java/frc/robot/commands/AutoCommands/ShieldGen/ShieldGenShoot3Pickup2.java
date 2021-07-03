// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ShieldGen;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.LimeLight;
import frc.robot.commands.AutoCommands.StartAllShooter;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShieldGenShoot3Pickup2 extends ParallelRaceGroup {
  /** Creates a new ShieldGenShoot3Pickup2. */
  public ShieldGenShoot3Pickup2(RevShooterSubsystem shooter, RevTiltSubsystem tilt,

      RevTurretSubsystem turret, RevDrivetrain drive, CellTransportSubsystem transport, LimeLight limelight,
      RearIntakeSubsystem intake, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new StartAllShooter(shooter, transport, 7.5),

        new ShieldGenAuto(shooter, drive, tilt, turret, transport, intake, limelight, compressor)

    );
  }
}
