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
import frc.robot.commands.Shooter.WaitTiltTurretLocked;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ToTrenchTarget extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        static double tiltAngle = ShootData.trench3M3BallShotConstants.tiltAngle;
        static double turretAngle = ShootData.trench3M3BallShotConstants.turretAngle;
        static double shootSpeed = ShootData.trench3M3BallShotConstants.shootSpeed;
        static double tiltOffset = ShootData.trench3M3BallShotConstants.tiltOffset;
        static double turretOffset = ShootData.trench3M3BallShotConstants.turretOffset;
        static double shootTime = ShootData.trench3M3BallShotConstants.shootTime;

        static double retractDistance = ShootData.trench4BallShotConstants.retractDistance;
        static double tiltAngle1 = ShootData.trench4BallShotConstants.tiltAngle;
        static double turretAngle1 = ShootData.trench4BallShotConstants.turretAngle;
        static double shootSpeed1 = ShootData.trench4BallShotConstants.shootSpeed;
        static double tiltOffset1 = ShootData.trench4BallShotConstants.tiltOffset;
        static double turretOffset1 = ShootData.trench4BallShotConstants.turretOffset;
        static double shootTime1 = ShootData.trench4BallShotConstants.shootTime;

        public ToTrenchTarget(RevTurretSubsystem turret, RevTiltSubsystem tilt, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(

                                // 1st lock
                                new ParallelCommandGroup(
                                                new SetUpLimelightForTarget(limelight, limelight.activeTrenchPipeline,
                                                                false),
                                                new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),
                                                new PositionTilt(tilt, tiltAngle + tiltOffset),
                                                new PositionTurret(turret, turretAngle + turretOffset)),
                                new UseVision(limelight, true), new WaitTiltTurretLocked(tilt, turret));

        }
}
