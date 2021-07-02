/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.AutoCommands.StartAllShooter;
import frc.robot.commands.Tilt.EndTiltLog;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.SetLogTiltItems;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.EndTurretLog;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.SetLogTurretItems;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShotPosition0 extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */

        public SetShotPosition0(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(

                                new ParallelCommandGroup(new SetActiveTeleopShootData(shooter, 0),
                                                new SetLogTiltItems(tilt, true), new SetLogTurretItems(turret, true),
                                                new SetLogShooterItems(shooter, true),
                                                new ChooseShooterSpeedSource(shooter, tilt, turret, 0),
                                                new SetUpLimelightForTarget(limelight),
                                                new SetTiltOffset(tilt, ShootData.centerPowerPortConstants.tiltOffset),
                                                new SetTurretOffset(turret,
                                                                ShootData.centerPowerPortConstants.turretOffset),
                                                new PositionTiltToVision(tilt, limelight,
                                                                ShootData.centerPowerPortConstants.tiltAngle
                                                                                + ShootData.centerPowerPortConstants.tiltOffset),
                                                new PositionTurretToVision(turret, limelight,
                                                                ShootData.centerPowerPortConstants.turretAngle
                                                                                + ShootData.centerPowerPortConstants.turretOffset),
                                                new SetShootSpeed(shooter,
                                                                ShootData.centerPowerPortConstants.shootSpeed),
                                                new UseVision(limelight, true),
                                                new StartAllShooter(shooter, transport, 2)),

                                new SetLogTiltItems(tilt, false), new SetLogTurretItems(turret, false),
                                new SetLogShooterItems(shooter, false), new EndTiltLog(tilt), new EndTurretLog(turret),
                                new EndShootLog(shooter));
        }
}