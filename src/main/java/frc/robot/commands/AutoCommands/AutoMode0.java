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
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Vision.LimelightSetPipeline;
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
     * Start in front of power port and shoot
     */

    // private final static int shootPosition = 0;
    // private final static double shootTime =
    // ShootData.getShootTime(shootPosition);
    // private final static int pipeline = ShootData.getPipeline(shootPosition);
    // private static double tiltAngle = ShootData.getTiltAngle(shootPosition);
    // private final static double turretAngle =
    // ShootData.getTurretAngle(shootPosition);
    // private final static double shootSpeed =
    // ShootData.getShootSpeed(shootPosition);
    // private final static double moveDistance =
    // ShootData.getMoveDistance(shootPosition);
    // private final static double shootDistance =
    // ShootData.getShootDistance(shootPosition);

    public AutoMode0(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
            CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight, Compressor compressor,
            int shootNumber) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());

        super(new LimelightSetPipeline(limelight, ShootData.getPipeline(shootNumber)),

                new ParallelCommandGroup(new PositionTiltToVision(tilt, limelight, ShootData.getTiltAngle(shootNumber)),
                        new PositionTurretToVision(turret, limelight, ShootData.getTurretAngle(shootNumber)),
                        new PositionRobot(drive, ShootData.getFirstDistance(shootNumber))
                                .deadlineWith(new StartShooterWheels(shooter, ShootData.getShootSpeed(shootNumber)))),

                new ParallelCommandGroup(new MessageCommand("Group2Started"),
                        new ShootCells(shooter, transport, compressor, ShootData.getShootSpeed(shootNumber),
                                ShootData.getShootTime(shootNumber))),

                new ParallelCommandGroup(new MessageCommand("GroupStarted"),
                        new PositionRobot(drive, ShootData.getSecondDistance(shootNumber)), new PositionTilt(tilt, 60),
                        new PositionTurret(turret, 0)));

    }
}
