// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestStuff;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class One extends CommandBase {
  /** Creates a new One. */
  int aa;
  double b;
  double c;

  public One() {
    // Use addRequirements() here to declare subsystem dependencies.
    int aa;
    double b;
    double c;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {aa=0;}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    aa++;
  
      
      SmartDashboard.putNumber("AA1", aa);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return aa>2002;
  }
}
