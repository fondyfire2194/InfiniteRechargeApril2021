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
import frc.robot.commands.CellIntake.IntakeArm;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.CellIntake.StopIntake;
import frc.robot.commands.CellIntake.StopIntakeMotor;
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
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
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
public class AutoModeShieldGen extends SequentialCommandGroup {
        /**
         * Creates a new Auto1.
         * 
         * Start in line with balls on shield generator, retract and shoot, retract wile
         * picking up then shoot
         */

        static double retractDistance = ShootData.shieldGenConstants.retractDistance;
        static double tiltAngle = ShootData.shieldGenConstants.tiltAngle;
        static double turretAngle = ShootData.shieldGenConstants.turretAngle;
        static double shootSpeed = ShootData.shieldGenConstants.shootSpeed;
        static double tiltOffset = ShootData.shieldGenConstants.tiltOffset;
        static double turretOffset = ShootData.shieldGenConstants.turretOffset;
        static double shootTime = ShootData.shieldGenConstants.shootTime;

        public AutoModeShieldGen(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                // move back and pickup 2
                super(new ResetEncoders(drive), new ResetGyro(drive),

                                new ParallelCommandGroup(new IntakeArm(intake, true), new RunIntakeMotor(intake, .75),
                                                new PickupMove(drive, retractDistance, -.5)),

                                new ParallelCommandGroup(new PositionProfiled(drive, -2, 3),
                                                new PositionTilt(tilt, tiltAngle + tiltOffset),
                                                new PositionTurret(turret, turretAngle + turretOffset)),

                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),
                                                new SetUpLimelightForTarget(limelight), new StopIntakeMotor(intake)),

                                new StartShooterWheels(shooter, shootSpeed),
                                new ParallelCommandGroup(new MessageCommand("Shoot1Started"),
                                                new StartShooterWheels(shooter, shootSpeed),
                                                new ShootCells(shooter, tilt, turret, limelight, transport, compressor,
                                                                shootTime)).deadlineWith(
                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),
                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight),
                                                                                new LogShootData(drive, turret, tilt,
                                                                                                shooter, limelight)),

                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"), new EndLogData(shooter),
                                                new StopShoot(shooter, transport), new IntakeArm(intake, false),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
                                                new SetUpLimelightForNoVision(limelight),
                                                new PositionTurret(turret, 0)));

        }
}