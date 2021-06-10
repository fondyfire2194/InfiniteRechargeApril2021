// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class RunRollers extends CommandBase {
  /** Creates a new RunRollers. */

  private final CellTransportSubsystem m_transport;

  public RunRollers(CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport=transport;
    addRequirements(m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_transport.startRollers){
    m_transport.runRearRollerMotor();
    m_transport.runFrontRollerMotor();
    } else{
      m_transport.stopFrontRollerMotor();
      m_transport.stopRearRollerMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
