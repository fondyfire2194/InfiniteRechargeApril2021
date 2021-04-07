// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class TurnCPWheel extends CommandBase {
  /** Creates a new TurnCPWheel. */
  private final ControlPanelSubsystem m_cp;
  private double m_speed;

  public TurnCPWheel(ControlPanelSubsystem cp, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cp = cp;
    m_speed = speed;
    addRequirements(m_cp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cp.turnWheelMotor(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cp.turnWheelMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
