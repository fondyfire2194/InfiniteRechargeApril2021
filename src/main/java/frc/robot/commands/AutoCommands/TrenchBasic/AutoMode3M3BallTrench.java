/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.TrenchBasic;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.CellIntake.StopIntakeMotor;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.SetTurretOffset;
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
public class AutoMode3M3BallTrench extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        static double tiltAngle = ShootData.trench3M3BallShotConstants.tiltAngle;
        static double turretAngle = ShootData.trench3M3BallShotConstants.turretAngle;
        static double shootSpeed = ShootData.trench3M3BallShotConstants.shootSpeed;
        static double tiltOffset = ShootData.trench3M3BallShotConstants.tiltOffset;
        static double turretOffset = ShootData.trench3M3BallShotConstants.turretOffset;
        static double shootTime = ShootData.trench3M3BallShotConstants.shootTime;

        static double retractDistance1 = ShootData.trench6BallShotConstants.retractDistance;
        static double tiltAngle1 = ShootData.trench6BallShotConstants.tiltAngle;
        static double turretAngle1 = ShootData.trench6BallShotConstants.turretAngle;
        static double shootSpeed1 = ShootData.trench6BallShotConstants.shootSpeed;
        static double tiltOffset1 = ShootData.trench6BallShotConstants.tiltOffset;
        static double turretOffset1 = ShootData.trench6BallShotConstants.turretOffset;
        static double shootTime1 = ShootData.trench6BallShotConstants.shootTime;

        public AutoMode3M3BallTrench(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(new ResetEncoders(drive), new ResetGyro(drive),

                                // 1st lock
                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),
                                                new SetUpLimelightForTarget(limelight), new UseVision(limelight, false),
                                                new PositionTiltToVision(tilt, limelight, tiltAngle + tiltOffset),
                                                new PositionTurretToVision(turret, limelight,
                                                                turretAngle + turretOffset)).deadlineWith(
                                                                                new IntakeArmLower(intake)),
                                // 1st Shoot
                                new ParallelCommandGroup(new MessageCommand("Shoot1Started"),
                                                new SetShootSpeed(shooter, shootSpeed), new UseVision(limelight, true),

                                                new ShootCells(shooter, tilt, turret, limelight, transport, compressor,
                                                                shootTime)).deadlineWith(
                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),
                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight)),
                                // 2nd pickup

                                new PickupMove(drive, retractDistance1, .7)

                                                .deadlineWith(new UseVision(limelight, false),
                                                                new SetTiltOffset(tilt, tiltOffset1),
                                                                new SetTurretOffset(turret, turretOffset1),

                                                                new PositionTilt(tilt, tiltAngle1 + tiltOffset1),
                                                                new PositionTurret(turret,
                                                                                turretAngle1 + turretOffset1),
                                                                new IntakeArmLower(intake),
                                                                new RunIntakeMotor(intake, .75)),

                                // // 2nd shoot
                                new ParallelCommandGroup(new MessageCommand("Shoot2Started"),
                                                new StopIntakeMotor(intake), new SetShootSpeed(shooter, shootSpeed1),
                                                new UseVision(limelight, true),

                                                new ShootCells(shooter, tilt, turret, limelight, transport, compressor,
                                                                shootTime)).deadlineWith(
                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),
                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight)),

                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"),
                                                // new StopShoot(shooter, transport),
                                                new IntakeArmRaise(intake), new StopIntakeMotor(intake),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
                                                new SetUpLimelightForNoVision(limelight),
                                                new PositionTurret(turret, 0)));

        }
}