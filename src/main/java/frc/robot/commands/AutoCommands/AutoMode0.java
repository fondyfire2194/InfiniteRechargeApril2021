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
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoMode0 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */
        static double retractDistance = ShootData.auto0Constants.retractDistance;
        static double tiltAngle = ShootData.auto0Constants.tiltAngle;
        static double turretAngle = ShootData.auto0Constants.turretAngle;
        static double shootSpeed = ShootData.auto0Constants.shootSpeed;
        static double tiltOffset = ShootData.auto0Constants.tiltOffset;
        static double turretOffset = ShootData.auto0Constants.turretOffset;
        static double shootTime = ShootData.auto0Constants.shootTime;

        public AutoMode0(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new ParallelCommandGroup(new LimelightSetPipeline(limelight, limelight.noZoomPipeline),
                                new UseVision(limelight, true), new StartShooterWheels(shooter, shootSpeed),
                                new SetTiltOffset(tilt, tiltOffset),
                                new PositionTiltToVision(tilt, limelight, tiltAngle),
                                new SetTurretOffset(turret, turretOffset),
                                new PositionTurretToVision(turret, limelight, turretAngle),
                                new PositionRobot(drive, retractDistance)),

                                new ParallelCommandGroup(new MessageCommand("ShootStarted"),
                                                new ShootCells(shooter, limelight, transport, compressor, shootTime)),

                                new ParallelCommandGroup(new MessageCommand("ReturnAxesStarted"),
                                                new StopShoot(shooter, transport),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MIN_ANGLE),
                                                new LimelightSetPipeline(limelight, limelight.driverPipeline),
                                                new UseVision(limelight, false), new PositionTurret(turret, 0)));

        }
}
