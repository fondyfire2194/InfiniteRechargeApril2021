// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LobShot extends SequentialCommandGroup {
  /** Creates a new LobShot. */
  public LobShot(RevShooterSubsystem shooter, RevTiltSubsystem tilt, LimeLight limelight,
      CellTransportSubsystem transport, Compressor compressor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimelightSetPipeline(limelight, 8), new PositionTilt(tilt, HoodedShooterConstants.TILT_MAX_ANGLE),
        new SetShooterSpeed(shooter, 1), new ShootCells(shooter, transport, compressor, 5));
  }
}
