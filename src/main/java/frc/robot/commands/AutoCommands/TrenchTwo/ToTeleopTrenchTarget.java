/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.TrenchTwo;

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
public class ToTeleopTrenchTarget extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        static double retractDistance = ShootData.trench5BallShotConstants.retractDistance;
        static double tiltAngle = ShootData.trench5BallShotConstants.tiltAngle;
        static double turretAngle = ShootData.trench5BallShotConstants.turretAngle;
        static double shootSpeed = ShootData.trench5BallShotConstants.shootSpeed;
        static double tiltOffset = ShootData.trench5BallShotConstants.tiltOffset;
        static double turretOffset = ShootData.trench5BallShotConstants.turretOffset;
        static double shootTime = ShootData.trench5BallShotConstants.shootTime;

        public ToTeleopTrenchTarget(RevTurretSubsystem turret, RevTiltSubsystem tilt, LimeLight limelight) {
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
