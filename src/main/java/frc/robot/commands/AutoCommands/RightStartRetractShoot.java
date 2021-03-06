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
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.PositionProfiled;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.EndLogData;
import frc.robot.commands.Shooter.LogShootData;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RightStartRetractShoot extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */
        static double retractDistance = ShootData.rightRetractShootConstants.retractDistance;
        static double tiltAngle = ShootData.rightRetractShootConstants.tiltAngle;
        static double turretAngle = ShootData.rightRetractShootConstants.turretAngle;
        static double shootSpeed = ShootData.rightRetractShootConstants.shootSpeed;
        static double tiltOffset = ShootData.rightRetractShootConstants.tiltOffset;
        static double turretOffset = ShootData.rightRetractShootConstants.turretOffset;
        static double shootTime = ShootData.rightRetractShootConstants.shootTime;

        public RightStartRetractShoot(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new ResetEncoders(drive), new ResetGyro(drive),
                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),
                                                new PickupMove(drive, -1, -.5),
                                                new PositionTilt(tilt, tiltAngle + tiltOffset),
                                                new PositionTurret(turret, turretAngle + turretOffset)),
                                new SetUpLimelightForTarget(limelight),
                                new ParallelCommandGroup(new MessageCommand("ShootIs3Started"),
                                                new StartShooterWheels(shooter, shootSpeed),
                                                new ShootCells(shooter, tilt, turret, limelight, transport, null, compressor,
                                                                shootTime)).deadlineWith(
                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),
                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight),
                                                                                new LogShootData(turret, tilt,
                                                                                                shooter, transport, limelight)),

                                new ParallelCommandGroup(new MessageCommand("ReturnAxesStarted"),
                                                new EndLogData(shooter), new StopShoot(shooter, transport),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
                                                new SetUpLimelightForNoVision(limelight)));
        }
}
