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
import frc.robot.commands.AutoCommands.StartAllShooter;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShotPosition1 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */

        public SetShotPosition1(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(

                                new ParallelCommandGroup(new SetActiveTeleopShootData(shooter, 1),
                                                new ChooseShooterSpeedSource(shooter, tilt, turret, 1),
                                                new SetUpLimelightForTarget(limelight),
                                                new SetTiltOffset(tilt, ShootData.trench4BallShotConstants.tiltOffset),
                                                new SetTurretOffset(turret,
                                                                ShootData.trench4BallShotConstants.turretOffset),
                                                new PositionTilt(tilt, ShootData.trench4BallShotConstants.tiltAngle
                                                                + ShootData.trench4BallShotConstants.tiltOffset),
                                                new PositionTurret(turret,
                                                                ShootData.trench4BallShotConstants.turretAngle
                                                                                + ShootData.trench4BallShotConstants.turretOffset),
                                                new SetShootSpeed(shooter,
                                                                ShootData.trench4BallShotConstants.shootSpeed),
                                                new StartAllShooter(shooter, transport, 2)));

        }
}