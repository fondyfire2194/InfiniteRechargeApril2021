// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.A3M2;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A3M31 extends SequentialCommandGroup {
  /** Creates a new A3M31. */

  static double tiltAngle = ShootData.trench3M3BallShotConstants.tiltAngle;
  static double turretAngle = ShootData.trench3M3BallShotConstants.turretAngle;
  static double shootSpeed = ShootData.trench3M3BallShotConstants.shootSpeed;
  static double tiltOffset = ShootData.trench3M3BallShotConstants.tiltOffset;
  static double turretOffset = ShootData.trench3M3BallShotConstants.turretOffset;

  public A3M31(RevTurretSubsystem turret, RevTiltSubsystem tilt, LimeLight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    super(
        new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset), new SetTurretOffset(turret, turretOffset),
            new PositionTurret(turret, turretAngle + turretOffset)),

        new ParallelCommandGroup(

            new SetUpLimelightForTarget(limelight), new UseVision(limelight, false),

            new PositionTiltToVision(tilt, limelight, tiltAngle + tiltOffset),
            
            new PositionTurretToVision(turret, limelight, turretAngle + turretOffset)));

  }
}