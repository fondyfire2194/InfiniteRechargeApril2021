// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ShieldGen;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.LimeLight;
import frc.robot.commands.CellTransport.RunBelts;
import frc.robot.commands.CellTransport.RunRollers;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.commands.AutoCommands.StartAllShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShieldGenAuto extends ParallelRaceGroup {
  /** Creates a new TestAuto. */
  public ShieldGenAuto(RevShooterSubsystem shooter, RevDrivetrain drive, RevTiltSubsystem tilt,
      RevTurretSubsystem turret, CellTransportSubsystem transport, RearIntakeSubsystem intake, LimeLight limelight,
      Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(new StartAllShooter(shooter, transport, .75), new RunRollers(transport),
            new RunBelts(transport)),

        new AutoModeShieldGen(shooter, turret, tilt, transport, drive, limelight, compressor, intake));

  }
}
