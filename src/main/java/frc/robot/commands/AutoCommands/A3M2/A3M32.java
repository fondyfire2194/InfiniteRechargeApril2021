// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.A3M2;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.CellIntake.StopIntakeMotor;
import frc.robot.commands.CellTransport.StartRollers;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooter;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A3M32 extends ParallelCommandGroup {
  /** Creates a new A3M32. */
  static double shootSpeed = ShootData.trench3M3BallShotConstants.shootSpeed;

  static double shootTime = ShootData.trench3M3BallShotConstants.shootTime;

  public A3M32(CellTransportSubsystem transport, RevShooterSubsystem shooter, RevTurretSubsystem turret,
      RevTiltSubsystem tilt, RearIntakeSubsystem intake, LimeLight limelight, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    super(new SetShootSpeed(shooter, shootSpeed), new StartRollers(transport, true, .75),

        new ParallelCommandGroup(new MessageCommand("Shoot1Started"), new StartShooter(shooter),
            new ShootCells(shooter, tilt, turret, limelight, transport, compressor, shootTime)).deadlineWith(
                new StopIntakeMotor(intake), new PositionHoldTilt(tilt, shooter, limelight),
                new PositionHoldTurret(turret, shooter, limelight))

    );
  }
}
