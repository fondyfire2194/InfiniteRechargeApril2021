// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.TrenchShootMoving;

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
public class ShootInMotionAuto extends ParallelRaceGroup {
  /** Creates a new Trench3BAllShootPlusPickup. */
  public ShootInMotionAuto(RevShooterSubsystem shooter, RevDrivetrain drive, RevTiltSubsystem tilt,
      RevTurretSubsystem turret, CellTransportSubsystem transport, RearIntakeSubsystem intake, LimeLight limelight,
      Compressor compressor) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

         new StartAllShooter(shooter, transport, .75),
       // new PulseBelts(transport),
        new AutoModeTrenchShootOnTheMove(shooter, turret, tilt, transport, drive, limelight, compressor, intake));
  }
}
