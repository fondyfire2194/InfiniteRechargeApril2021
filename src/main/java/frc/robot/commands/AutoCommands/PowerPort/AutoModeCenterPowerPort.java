/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.PowerPort;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.StopIntake;
import frc.robot.commands.CellTransport.SetLeftReleaseShots;
import frc.robot.commands.RobotDrive.PickupMoveVelocity;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.EndShootLog;
import frc.robot.commands.Shooter.SetLogShooterItems;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.SetShotPositionPowerPort;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetLogTiltItems;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.SetLogTurretItems;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoModeCenterPowerPort extends SequentialCommandGroup {
        /**
         * Creates a new Center Auto
         * 
         * Start in front of power port, retract and shoot
         */
        static double retractDistance = ShootData.centerPowerPortConstants.retractDistance;
        static double tiltAngle = ShootData.centerPowerPortConstants.tiltAngle;
        static double turretAngle = ShootData.centerPowerPortConstants.turretAngle;
        static double shootSpeed = ShootData.centerPowerPortConstants.shootSpeed;
        static double tiltOffset = ShootData.centerPowerPortConstants.tiltOffset;
        static double turretOffset = ShootData.centerPowerPortConstants.turretOffset;
        static double shootTime = ShootData.centerPowerPortConstants.shootTime;

        public AutoModeCenterPowerPort(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, RearIntakeSubsystem intake,
                        LimeLight limelight, Compressor compressor) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new ResetEncoders(drive), new ResetGyro(drive), new PickupMoveVelocity(drive, -1, 2),

                                new ParallelCommandGroup(new SetLogTiltItems(tilt, true),

                                                new SetLogTurretItems(turret, true),

                                                new ToPowerPortTarget(turret, tilt, shooter, limelight),

                                                new SetLeftReleaseShots(transport, 2),

                                                new SetLogShooterItems(shooter, true),

                                                new SetShootSpeed(shooter, shootSpeed)),

                                new ShootCells(shooter, tilt, turret, limelight, transport, drive,

                                                compressor, shootTime).deadlineWith(

                                                                new PositionHoldTilt(tilt, shooter, limelight),

                                                                new PositionHoldTurret(turret, shooter, limelight)),

                                // s_trajectory.getRamsete(s_trajectory.centerStart).andThen(() ->
                                // drive.tankDriveVolts(0, 0)),

                                new ParallelCommandGroup(new MessageCommand("ReturnAxesStarted"),
                                                new SetLogTiltItems(tilt, false), new EndShootLog(shooter),
                                                new StopIntake(intake),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
                                                new SetUpLimelightForNoVision(limelight)));
        }
}
