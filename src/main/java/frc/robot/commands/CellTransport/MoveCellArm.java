// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

public class MoveCellArm extends CommandBase {
  /** Creates a new MoveCellArm. */
  private final CellTransportSubsystem m_transport;
  private double m_position;

  private int loopCtr;

  public MoveCellArm(CellTransportSubsystem transport, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_position = position;

  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.moveCellArm(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
