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
import frc.robot.commands.TimeDelay;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.EndLogData;
import frc.robot.commands.Shooter.ShootInMotion;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.EndTiltLog;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
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
public class AutoMode5BallTrench extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */
        static double retractDistance = ShootData.trench3M3BallShotConstants.retractDistance;
        static double tiltAngle = ShootData.trench5BallShotConstants.tiltAngle;
        static double turretAngle = ShootData.trench5BallShotConstants.turretAngle;
        static double shootSpeed = ShootData.trench5BallShotConstants.shootSpeed;
        static double tiltOffset = ShootData.trench5BallShotConstants.tiltOffset;
        static double turretOffset = ShootData.trench5BallShotConstants.turretOffset;
        static double shootTime = ShootData.trench5BallShotConstants.shootTime;

        public AutoMode5BallTrench(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(new ResetEncoders(drive), new ResetGyro(drive),

                                new ParallelCommandGroup(new SetTurretOffset(turret, turretOffset),
                                                new PositionTurretToVision(turret, limelight,
                                                                turretAngle + turretOffset),
                                                new SetTiltOffset(tilt, tiltOffset),
                                                new PositionTiltToVision(tilt, limelight, tiltAngle + tiltOffset),
                                                new UseVision(limelight, true)),

                                new ParallelCommandGroup(new PickupMove(drive, retractDistance, .25), new TimeDelay(15))
                                                .deadlineWith(new ParallelCommandGroup(
                                                                new ShootInMotion(shooter, tilt, turret, limelight,
                                                                                transport, compressor, shootTime),
                                                                new IntakeArmLower(intake),
                                                                new RunIntakeMotor(intake, .75))),

                                new ParallelCommandGroup(new MessageCommand("EndResetStarted"), new EndLogData(shooter),
                                                new EndTiltLog(tilt), new StopShoot(shooter, transport),
                                                new IntakeArmRaise(intake),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
                                                new SetUpLimelightForNoVision(limelight),
                                                new PositionTurret(turret, 0)));

        }
}
