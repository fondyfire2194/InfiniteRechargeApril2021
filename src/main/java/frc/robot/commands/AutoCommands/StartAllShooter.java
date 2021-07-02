// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.CellTransport.PulseBelts;
import frc.robot.commands.CellTransport.RunBelts;
import frc.robot.commands.CellTransport.RunRollers;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartAllShooter extends SequentialCommandGroup {
  /** Creates a new StartAllShooter. */
  public StartAllShooter(RevShooterSubsystem shooter, CellTransportSubsystem transport, double delay) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new TimeDelay(delay),

        new ParallelCommandGroup(new RunShooter(shooter), new RunRollers(transport), new PulseBelts(transport))

    );
  }
}
