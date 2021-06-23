// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RearIntakeSubsystem;

public class RunIntakeMotor extends CommandBase {
  /** Creates a new RunIntakeMotor. */
  private RearIntakeSubsystem m_intake;
  private double m_speed;

  public RunIntakeMotor(RearIntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runIntakeMotor(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
