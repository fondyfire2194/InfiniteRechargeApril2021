/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
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
public class LockAndShoot extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */
        static double tiltAngle = ShootData.activeTeleopTiltAngle;
        static double turretAngle = ShootData.activeTeleopTurretAngle;
        static double shootSpeed = ShootData.activeTeleopShootSpeed;
        static double tiltOffset = ShootData.activeTeleopTiltOffset;
        static double turretOffset = ShootData.activeTeleopTurretOffset;

        public LockAndShoot(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new ParallelCommandGroup(new LimelightSetPipeline(limelight, limelight.noZoomPipeline),
                                new UseVision(limelight, true),
                                new StartShooterWheels(shooter, shooter.teleopSetupShooterSpeed),
                                new SetTiltOffset(tilt, tiltOffset), new PositionTilt(tilt, tiltAngle),
                                new SetTurretOffset(turret, turretOffset), new PositionTurret(turret, turretAngle)),

                                new ParallelCommandGroup(new MessageCommand("ShootStarted"),
                                                new ShootCells(shooter, tilt, turret, limelight, transport, compressor, 0)),

                                new ParallelCommandGroup(new MessageCommand("ReturnAxesStarted"),
                                                new StopShoot(shooter, transport),
                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MIN_ANGLE),
                                                new LimelightSetPipeline(limelight, limelight.driverPipeline),
                                                new UseVision(limelight, false), new PositionTurret(turret, 0)));

        }
}
