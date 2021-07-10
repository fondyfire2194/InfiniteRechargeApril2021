// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CellTransportSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReleaseLeftArm extends InstantCommand {
  private CellTransportSubsystem m_transport;

  public ReleaseLeftArm(CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport=transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (!m_transport.getBallAtShoot())
      m_transport.releaseLeftChannel();
  }
}
