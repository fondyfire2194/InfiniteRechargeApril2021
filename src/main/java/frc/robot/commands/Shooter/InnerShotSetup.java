// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InnerShotSetup extends ParallelCommandGroup {
  /** Creates a new LobShot1. */
  public InnerShotSetup(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimelightSetPipeline(limelight, limelight.noZoomPipeline), new UseVision(limelight, true),
        new StartShooterWheels(shooter, ShootData.innerShotMPS),
        new PositionTiltToVision(tilt, limelight, ShootData.innerTiltAngle),
        new PositionTurretToVision(turret, limelight, 0));
  }
}
