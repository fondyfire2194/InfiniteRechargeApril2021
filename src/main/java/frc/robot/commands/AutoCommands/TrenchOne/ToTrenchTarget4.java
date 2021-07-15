/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.TrenchOne;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.AutoCommands.StartAllShooter;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.WaitTiltTurretLocked;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ToTrenchTarget4 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */
        static double retractDistance1 = ShootData.trench4Ball[0];
        static double tiltAngle1 = ShootData.trench4Ball[1];
        static double turretAngle1 = ShootData.trench4Ball[2];
        static double shootSpeed1 = ShootData.trench4Ball[3];
        static double tiltOffset1 = ShootData.trench4Ball[4];
        static double turretOffset1 = ShootData.trench4Ball[5];
        static double shootTime1 = ShootData.trench4Ball[6];

        public ToTrenchTarget4(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevShooterSubsystem shooter,
                        CellTransportSubsystem transport, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(

                                // 1st lock
                                new ParallelCommandGroup(
                                                new SetUpLimelightForTarget(limelight, limelight.activeTrenchPipeline,
                                                                false),
                                                new SetShootSpeed(shooter, shootSpeed1),
                                                new SetTiltOffset(tilt, tiltOffset1),
                                                new SetTurretOffset(turret, turretOffset1),
                                                new PositionTilt(tilt, tiltAngle1 + tiltOffset1),
                                                new PositionTurret(turret, turretAngle1 + turretOffset1)),
                                new UseVision(limelight, true), new WaitTiltTurretLocked(tilt, turret),

                                new StartAllShooter(shooter, transport, 0));

        }
}
