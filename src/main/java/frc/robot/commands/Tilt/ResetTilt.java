/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevTiltSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ResetTilt extends InstantCommand {
  private final RevTiltSubsystem tilt;

  public ResetTilt(RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tilt.resetAngle(0);
  }
}
