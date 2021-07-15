// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.TrenchTwo;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.AutoCommands.SetActive2ndShootData;
import frc.robot.commands.AutoCommands.StartAllShooter;
import frc.robot.commands.CellTransport.RunRollers;
import frc.robot.commands.Shooter.LogShootData;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrenchAuto2 extends ParallelRaceGroup {
  /** Creates a new Trench3BAllShootPlusPickup. */
  public TrenchAuto2(RevShooterSubsystem shooter, RevDrivetrain drive, RevTiltSubsystem tilt, RevTurretSubsystem turret,
      CellTransportSubsystem transport, RearIntakeSubsystem intake, LimeLight limelight, Compressor compressor) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

        new RunRollers(transport), new RunShooter(shooter), new LogShootData(shooter, transport, drive),

        new SetActive2ndShootData(ShootData.trench5Ball),

        new AutoMode3MnBallTrench(shooter, turret, tilt, transport, drive, limelight, compressor, intake));
  }
}
