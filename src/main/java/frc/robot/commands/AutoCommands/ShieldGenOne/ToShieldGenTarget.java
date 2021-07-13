/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.ShieldGenOne;

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
public class ToShieldGenTarget extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * 
         */

        static double tiltAngle = ShootData.shieldGen3MxBallShotConstants.tiltAngle;
        static double turretAngle = ShootData.shieldGen3MxBallShotConstants.turretAngle;
        static double shootSpeed = ShootData.shieldGen3MxBallShotConstants.shootSpeed;
        static double tiltOffset = ShootData.shieldGen3MxBallShotConstants.tiltOffset;
        static double turretOffset = ShootData.shieldGen3MxBallShotConstants.turretOffset;
        static double shootTime = ShootData.shieldGen3MxBallShotConstants.shootTime;

        static double retractDistance = ShootData.shieldGen4BallShotConstants.retractDistance;
        static double tiltAngle1 = ShootData.shieldGen4BallShotConstants.tiltAngle;
        static double turretAngle1 = ShootData.shieldGen4BallShotConstants.turretAngle;
        static double shootSpeed1 = ShootData.shieldGen4BallShotConstants.shootSpeed;
        static double tiltOffset1 = ShootData.shieldGen4BallShotConstants.tiltOffset;
        static double turretOffset1 = ShootData.shieldGen4BallShotConstants.turretOffset;
        static double shootTime1 = ShootData.shieldGen4BallShotConstants.shootTime;

        public ToShieldGenTarget(RevTurretSubsystem turret, RevTiltSubsystem tilt, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(

                                new SetUpLimelightForTarget(limelight, limelight.activeShieldGenPipeline, false),
                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),

                                                new PositionTilt(tilt, tiltAngle + tiltOffset),
                                                new PositionTurret(turret, turretAngle + turretOffset)),

                                new UseVision(limelight, true), new WaitTiltTurretLocked(tilt, turret));

        }
}
