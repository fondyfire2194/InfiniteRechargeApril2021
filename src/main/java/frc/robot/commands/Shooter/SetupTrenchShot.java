// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupTrenchShot extends ParallelCommandGroup {
  /** Creates a new LobShot1. */
  public SetupTrenchShot(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetTeleopSetupNumber(shooter, 2),
        new SetTeleopShooterSpeed(shooter, ShootData.trenchShotConstants.shootSpeed1),
        new SetTiltOffset(tilt, ShootData.trenchShotConstants.tiltOffset),
        new PositionTilt(tilt, ShootData.trenchShotConstants.tiltAngle),
        new SetTurretOffset(turret, ShootData.trenchShotConstants.turretOffset1),
        new PositionTurret(turret, ShootData.trenchShotConstants.turretAngle1));
  }
}
