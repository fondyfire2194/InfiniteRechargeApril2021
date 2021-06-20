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
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.CellIntake.StartIntake;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Tilt.TiltSeekVision;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoModeTrench extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */
        static double retractDistance = ShootData.trenchShotConstants.retractDistance;
        static double tiltAngle = ShootData.trenchShotConstants.tiltAngle;
        static double turretAngle = ShootData.trenchShotConstants.turretAngle;
        static double shootSpeed = ShootData.trenchShotConstants.shootSpeed;
        static double tiltOffset = ShootData.trenchShotConstants.tiltOffset;
        static double turretOffset = ShootData.trenchShotConstants.turretOffset;
        static double shootTime = ShootData.trenchShotConstants.shootTime;

        public AutoModeTrench(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(new ParallelCommandGroup(new ParallelCommandGroup(new StartIntake(intake, transport),
                                new PositionRobot(drive, -3, 2), new PositionTurret(turret, -turretAngle))),

                                new ParallelCommandGroup(new LimelightSetPipeline(limelight, limelight.noZoomPipeline),
                                                new UseVision(limelight, true), new TiltSeekVision(tilt, limelight)),

                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),
                                                new StartShooterWheels(shooter, shootSpeed)),
                                new ParallelCommandGroup(new MessageCommand("Shoot1Started"),
                                                new ShootCells(shooter, tilt, turret, limelight, transport, compressor, shootTime)),

                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"),
                                                new StopShoot(shooter, transport),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MIN_ANGLE),
                                                new LimelightSetPipeline(limelight, limelight.driverPipeline),
                                                new UseVision(limelight, false), new PositionTurret(turret, 0)));

        }
}
