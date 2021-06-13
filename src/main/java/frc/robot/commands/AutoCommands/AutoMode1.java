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
import frc.robot.commands.CellIntake.StopIntake;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
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
public class AutoMode1 extends SequentialCommandGroup {
        /**
         * Creates a new Auto1.
         * 
         * Start in line with balls on shield generator, retract and shoot, retract wile
         * picking up then shoot
         */

        static double retractDistance = ShootData.auto1Constants.retractDistance;
        static double tiltAngle = ShootData.auto1Constants.tiltAngle;
        static double turretAngle = ShootData.auto1Constants.turretAngle;
        static double shootSpeed = ShootData.auto1Constants.shootSpeed;
        static double tiltOffset = ShootData.auto1Constants.tiltOffset;
        static double turretOffset = ShootData.auto1Constants.turretOffset;
        static double shootTime = ShootData.auto1Constants.shootTime;

        static double retractDistance1 = ShootData.auto1Constants.retractDistance1;
        static double tiltAngle1 = ShootData.auto1Constants.tiltAngle1;
        static double turretAngle1 = ShootData.auto1Constants.turretAngle1;
        static double shootSpeed1 = ShootData.auto1Constants.shootSpeed1;
        static double tiltOffset1 = ShootData.auto1Constants.tiltOffset1;
        static double turretOffset1 = ShootData.auto1Constants.turretOffset1;
        static double shootTime1 = ShootData.auto1Constants.shootTime1;

        public AutoMode1(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                // move back and pickup 2
                super(new ParallelCommandGroup(new LimelightSetPipeline(limelight, limelight.noZoomPipeline),
                                new UseVision(limelight, true), new StartShooterWheels(shooter, shootSpeed),
                                new SetTiltOffset(tilt, tiltOffset),
                                new PositionTiltToVision(tilt, limelight, tiltAngle),
                                new SetTurretOffset(turret, turretOffset),
                                new PositionTurretToVision(turret, limelight, turretAngle),
                                new PositionRobot(drive, retractDistance)),

                                new ParallelCommandGroup(new MessageCommand("Shoot1Started"),
                                                new ShootCells(shooter, limelight, transport, compressor, shootTime)),

                                new ParallelCommandGroup(new MessageCommand("Pickup Started"), new StartIntake(intake,transport),
                                                new PositionRobot(drive, retractDistance1),
                                                new StartShooterWheels(shooter, shootSpeed1),
                                                new SetTiltOffset(tilt, tiltOffset1),
                                                new PositionTiltToVision(tilt, limelight, tiltAngle1),
                                                new SetTurretOffset(turret, turretOffset1),
                                                new PositionTurretToVision(turret, limelight, turretAngle1)),

                                new ParallelCommandGroup(new MessageCommand("Shoot2Started"),
                                                new StartShooterWheels(shooter, shootSpeed1),
                                                new ShootCells(shooter, limelight, transport, compressor, shootTime),
                                                new StopIntake(intake)),

                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"),
                                                new StopShoot(shooter, transport),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MIN_ANGLE),
                                                new LimelightSetPipeline(limelight, limelight.driverPipeline),
                                                new UseVision(limelight, false), new PositionTurret(turret, 0)));

        }
}
