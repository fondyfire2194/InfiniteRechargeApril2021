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
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShieldGenShot extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */

        public ShieldGenShot(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, LimeLight limelight, Compressor compressor) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new LimelightSetPipeline(limelight, ShootData.getPipeline(2)),

                                new ParallelCommandGroup(
                                                new PositionTiltToVision(tilt, limelight, ShootData.getTiltAngle(2)),

                                                new PositionTurretToVision(turret, limelight,
                                                                ShootData.getTurretAngle(2)))
                                                                                .deadlineWith(new RunShooter(shooter)),

                                new ParallelCommandGroup(new MessageCommand("Group2Started"),
                                                new ShootCells(shooter, transport, compressor,
                                                                ShootData.getShootTime(2))),

                                new ParallelCommandGroup(new MessageCommand("GroupStarted"),

                                                new PositionTilt(tilt, HoodedShooterConstants.TILT_MIDFIELD_ANGLE),
                                                new LimelightSetPipeline(limelight, 8), new PositionTurret(turret, 0)));

        }
}
