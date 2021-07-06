// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class RaiseLeftArm extends CommandBase {
  /** Creates a new MoveCellArm. */
  private final CellTransportSubsystem m_transport;

  private int loopCr;

  public RaiseLeftArm(CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;

  };

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.releaseLeftChannel();
    loopCr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopCr++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopCr > 0;
  }
}
