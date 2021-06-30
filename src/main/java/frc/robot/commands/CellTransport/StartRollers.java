// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CellTransportSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartRollers extends InstantCommand {
  private CellTransportSubsystem m_transport;
  private double m_speed;
  private boolean m_state;

  public StartRollers(CellTransportSubsystem transport, boolean state, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
    m_speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_transport.startRollers = m_state;
    m_transport.rollerSpeed = m_speed;
  }
}
