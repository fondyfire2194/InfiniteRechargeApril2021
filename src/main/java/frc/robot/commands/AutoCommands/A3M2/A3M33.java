// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.A3M2;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A3M33 extends ParallelCommandGroup {
  /** Creates a new A3M32. */
  static double shootSpeed = ShootData.trench3M3BallShotConstants.shootSpeed;

  static double shootTime = ShootData.trench3M3BallShotConstants.shootTime;

  public A3M33(RevDrivetrain drive, RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      RearIntakeSubsystem intake, LimeLight limelight, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(new SetShootSpeed(shooter, shootSpeed / 2), new SetUpLimelightForNoVision(limelight),

        new ParallelCommandGroup(new IntakeArmLower(intake), new RunIntakeMotor(intake, .75),
            new PositionTurret(turret, 0), new PositionTilt(tilt, 0), new PickupMove(drive, -3.5, .6, .1))

    );
  }
}
