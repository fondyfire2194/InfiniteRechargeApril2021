// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CellTransportSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLeftReleaseShots extends InstantCommand {
  private CellTransportSubsystem m_transport;
  private int m_number;

  public SetLeftReleaseShots(CellTransportSubsystem transport, int number) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_number = number;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.cellsToBeShot = m_number;
  }
}
