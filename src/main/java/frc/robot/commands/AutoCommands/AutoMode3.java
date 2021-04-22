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
import frc.robot.commands.Shooter.ReturnTiltTurret;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShooterWheels;
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
public class AutoMode3 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        public AutoMode3(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake, int shootNumber) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                // move back and pickup 2

                super(new ParallelCommandGroup(new PositionRobot(drive, ShootData.getFirstDistance(shootNumber)),
                                new PositionTurretToVision(turret, limelight, ShootData.getTurretAngle(shootNumber)),
                                new PositionTiltToVision(tilt, limelight, ShootData.getTiltAngle(shootNumber)))

                                                .deadlineWith(new StartIntake(intake),
                                                                new StartShooterWheels(shooter,
                                                                                ShootData.getShootSpeed(shootNumber)),
                                                                new CalculateTargetDistance(limelight, tilt, shooter)),

                                // shoot 5

                                new ParallelCommandGroup(new StopIntake(intake),
                                                new ShootCells(shooter, transport, compressor,
                                                                ShootData.getShootSpeed(shootNumber),
                                                                ShootData.getShootTime(shootNumber))),

                                // pick up 3 move back under control panel
                                new ParallelCommandGroup(
                                                new PositionRobot(drive, ShootData.getFirstDistance(shootNumber + 1)),
                                                new ReturnTiltTurret(turret, 65, tilt, 0))

                                                                .deadlineWith(new StartIntake(intake),
                                                                                new StopShooterWheels(shooter)),

                                // move under control panel again

                                new SequentialCommandGroup(
                                                new PositionRobot(drive, ShootData.getFirstDistance(shootNumber + 1)),
                                                new StopIntake(intake)),

                                new ParallelCommandGroup(
                                                new PositionTurretToVision(turret, limelight,
                                                                ShootData.getTurretAngle(shootNumber)),
                                                new PositionTiltToVision(tilt, limelight,
                                                                ShootData.getTiltAngle(shootNumber)))

                                                                                .deadlineWith(new StartShooterWheels(
                                                                                                shooter,
                                                                                                ShootData.getShootSpeed(
                                                                                                                shootNumber)),
                                                                                                new CalculateTargetDistance(
                                                                                                                limelight,
                                                                                                                tilt,
                                                                                                                shooter)),

                                // shoot 3

                                new ShootCells(shooter, transport, compressor, ShootData.getShootSpeed(shootNumber + 1),
                                                ShootData.getShootTime(shootNumber + 1)));
        }

}
