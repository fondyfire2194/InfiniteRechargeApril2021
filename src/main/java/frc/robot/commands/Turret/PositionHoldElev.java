// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevElevatorSubsystem;

public class PositionHoldElev extends CommandBase {
  /** Creates a new PositionTilt. */

  private final RevElevatorSubsystem m_elev;;

  private double m_position;

  
  public PositionHoldElev(RevElevatorSubsystem elev) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elev = elev;
    

    addRequirements(m_elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_position = m_elev.getHeightInches();
    SmartDashboard.putNumber("ElPOS", m_position);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elev.goToPosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
