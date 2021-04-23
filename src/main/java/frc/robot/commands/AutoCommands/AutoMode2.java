/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.CellIntake.StartIntake;
import frc.robot.commands.CellIntake.StopIntake;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoMode2 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        public AutoMode2(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake, int shootNumber) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                // move back and pickup 2

                super(new ParallelCommandGroup(new PositionRobot(drive, ShootData.getFirstDistance(shootNumber)),
                                new PositionTurretToVision(turret, limelight, ShootData.getTurretAngle(shootNumber)),
                                new PositionTiltToVision(tilt, limelight, ShootData.getTiltAngle(shootNumber)))

                                                .deadlineWith(new StartIntake(intake,limelight),
                                                                new StartShooterWheels(shooter,
                                                                                ShootData.getShootSpeed(shootNumber)),
                                                                new CalculateTargetDistance(limelight, tilt, shooter)),

                                // shoot 5

                                new ParallelCommandGroup(new StopIntake(intake),
                                                new ShootCells(shooter, transport, compressor,
                                                                ShootData.getShootSpeed(shootNumber),
                                                                ShootData.getShootTime(shootNumber))),
                                // pick up 1
                                new ParallelCommandGroup(
                                                new PositionRobot(drive, ShootData.getFirstDistance(shootNumber + 1)),
                                                new PositionTurretToVision(turret, limelight,
                                                                ShootData.getTurretAngle(shootNumber + 1)),
                                                new PositionTiltToVision(tilt, limelight,
                                                                ShootData.getTiltAngle(shootNumber + 1)))

                                                                                .deadlineWith(new StartIntake(intake,limelight),
                                                                                                new StartShooterWheels(
                                                                                                                shooter,
                                                                                                                ShootData.getShootSpeed(
                                                                                                                                shootNumber + 1)),
                                                                                                new CalculateTargetDistance(
                                                                                                                limelight,
                                                                                                                tilt,
                                                                                                                shooter)),

                                // shoot 1

                                new ShootCells(shooter, transport, compressor, ShootData.getShootSpeed(shootNumber + 1),
                                                ShootData.getShootTime(shootNumber + 1)));
        }

}
