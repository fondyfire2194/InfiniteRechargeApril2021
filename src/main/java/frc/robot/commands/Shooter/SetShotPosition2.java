/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShotPosition2 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */

        public SetShotPosition2(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new SetActiveTeleopShootData(shooter, 2), new SetUpLimelightForTarget(limelight),
                                new TimeDelay(1.),

                                new ParallelCommandGroup(
                                                new SetTiltOffset(tilt, ShootData.trench5BallShotConstants.tiltOffset),
                                                new SetTurretOffset(turret,
                                                                ShootData.trench5BallShotConstants.turretOffset),
                                                new PositionTiltToVision(tilt, limelight,
                                                                ShootData.trench5BallShotConstants.tiltAngle
                                                                                + ShootData.trench5BallShotConstants.tiltOffset),
                                                new PositionTurretToVision(turret, limelight,
                                                                ShootData.trench5BallShotConstants.turretAngle
                                                                                + ShootData.trench5BallShotConstants.turretOffset)),
                                new SetShootSpeed(shooter, ShootData.trench5BallShotConstants.shootSpeed),
                                new UseVision(limelight, true), new StartShooter(shooter));
        }
}