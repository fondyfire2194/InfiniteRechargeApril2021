// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.commands.MessageCommand;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Vision.LimelightLeds;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootStraight extends SequentialCommandGroup {

  /** Creates a new ShootStraight. */
  public ShootStraight(RevTiltSubsystem tilt, LimeLight limelight, RevShooterSubsystem shooter, RevDrivetrain drive,
      CellTransportSubsystem transport, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    super(new LimelightSetPipeline(limelight, 0),

        new ParallelCommandGroup(new PositionTiltToVision(tilt, limelight, HoodedShooterConstants.TILT_CLOSE_ANGLE, 0),
            new OKToShoot(shooter, limelight, drive)).deadlineWith(new StartShooterWheels(shooter)),

        new ParallelCommandGroup(new MessageCommand("Group2Started"), new ShootCells(shooter, transport, compressor)),

        new ParallelCommandGroup(new MessageCommand("GroupStarted"),

            new PositionTilt(tilt, HoodedShooterConstants.TILT_MIDFIELD_ANGLE),
            
            new LimelightSetPipeline(limelight, 8)));

  }
}
