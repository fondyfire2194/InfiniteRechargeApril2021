/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.TrenchTwo;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.CellIntake.StopIntakeMotor;
import frc.robot.commands.CellTransport.SetLeftReleaseShots;
import frc.robot.commands.RobotDrive.PickupMoveVelocity;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.SetLogShooterItems;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.WaitTiltTurretLocked;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
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
public class AutoMode3MnBallTrench extends SequentialCommandGroup {
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

        static double retractDistance1 = ShootData.activeValues[0];
        static double tiltAngle1 = ShootData.activeValues[1];
        static double turretAngle1 = ShootData.activeValues[2];
        static double shootSpeed1 = ShootData.activeValues[3];
        static double tiltOffset1 = ShootData.activeValues[4];
        static double turretOffset1 = ShootData.activeValues[5];
        static double shootTime1 = ShootData.activeValues[6];

        public AutoMode3MnBallTrench(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //
                // 1st lock
                super(new ParallelCommandGroup(new ResetEncoders(drive), new ResetGyro(drive),
                                new SetLeftReleaseShots(transport, 2), new SetShootSpeed(shooter, shootSpeed),
                                new ToTrenchTarget(turret, tilt, shooter, limelight))
                                                .deadlineWith(new IntakeArmLower(intake)),

                                // 1st Shoot
                                new ParallelCommandGroup(new MessageCommand("Shoot1Started"),

                                                new ShootCells(shooter, tilt, turret, limelight, transport, drive,
                                                                compressor, shootTime)).deadlineWith(
                                                                                new IntakeArmLower(intake),
                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),
                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight)),
                                // 2nd pickup

                                new PickupMoveVelocity(drive, retractDistance1, 1.5)

                                                .deadlineWith(new SetLeftReleaseShots(transport, 1),

                                                                new UseVision(limelight, false),

                                                                new SetTiltOffset(tilt, tiltOffset1),

                                                                new SetTurretOffset(turret, turretOffset1),

                                                                new PositionTilt(tilt, tiltAngle1 + tiltOffset1),

                                                                new PositionTurret(turret,
                                                                                turretAngle1 + turretOffset1),
                                                                new IntakeArmLower(intake),

                                                                new SetShootSpeed(shooter, shootSpeed1),

                                                                new RunIntakeMotor(intake, .75)),

                                new UseVision(limelight, true),

                                new WaitTiltTurretLocked(tilt, turret).deadlineWith(
                                                new PositionHoldTilt(tilt, shooter, limelight),
                                                new PositionHoldTurret(turret, shooter, limelight)),

                                // // 2nd shoot
                                new ParallelCommandGroup(new MessageCommand("Shoot2Started"),

                                                new ShootCells(shooter, tilt, turret, limelight, transport, drive,
                                                                compressor, shootTime).deadlineWith(

                                                                                new PositionHoldTilt(tilt, shooter,
                                                                                                limelight),

                                                                                new PositionHoldTurret(turret, shooter,
                                                                                                limelight)),

                                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"),
                                                                new SetLogShooterItems(shooter, false),
                                                                new IntakeArmRaise(intake), new StopIntakeMotor(intake),
                                                                new PositionTilt(tilt,
                                                                                HoodedShooterConstants.TILT_MAX_ANGLE),
                                                                new SetUpLimelightForNoVision(limelight),
                                                                new PositionTurret(turret, 0))));

        }
}
