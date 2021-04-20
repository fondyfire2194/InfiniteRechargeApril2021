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
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionHoldTurret;
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
public class AutoMode3 extends SequentialCommandGroup {
  /**
   * Creates a new Auto3
   * 
   * Start left of power port move back and shoot
   */

  private final static int shootPosition = 3;
  private final static double shootTime = ShootData.getShootTime(shootPosition);
  private final static int pipeline = ShootData.getPipeline(shootPosition);
  private static double tiltAngle = ShootData.getTiltAngle(shootPosition);
  private final static double turretAngle = ShootData.getTurretAngle(shootPosition);
  private final static double shootSpeed = ShootData.getShootSpeed(shootPosition);
  private final static double moveDistance = ShootData.getDistance(shootPosition);

  public AutoMode3(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight, Compressor compressor) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(new LimelightSetPipeline(limelight, pipeline),

        new ParallelCommandGroup(new PositionTiltToVision(tilt, limelight, tiltAngle),
            new PositionTurretToVision(turret, limelight, turretAngle),
            new PositionRobot(drive, moveDistance).deadlineWith(new StartShooterWheels(shooter, shootSpeed))),

        new ParallelCommandGroup(new ShootCells(shooter, transport, compressor, shootSpeed, shootTime)
            .deadlineWith(new ParallelCommandGroup(new PositionHoldTilt(tilt)), new PositionHoldTurret(turret))),

        new ParallelCommandGroup(new PositionTilt(tilt, -1), new PositionTurret(turret, 0)));

  }
}
