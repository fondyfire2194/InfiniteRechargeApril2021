// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class ReleaseOneCell extends CommandBase {
  /** Creates a new ReleaseOneCell. */
  private final CellTransportSubsystem m_transport;
  private double startTime;

  public ReleaseOneCell(CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transport = transport;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    m_transport.moveCellArm(m_transport.cellArmReleaseCell);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > startTime + m_transport.cellReleaseTime) {
      m_transport.moveCellArm(m_transport.cellArmHoldCell);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > (startTime + m_transport.cellReleaseTime +.1);
  }
}
