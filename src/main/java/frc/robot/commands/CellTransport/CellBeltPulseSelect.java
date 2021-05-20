// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellTransport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class CellBeltPulseSelect extends CommandBase {
  /** Creates a new CellBeltPulseSelect. */
  private final CellTransportSubsystem m_transport;
  private double m_firstSpeed;
  private double m_firstTime;
  private double m_secondSpeed;
  private double m_secondTime;
  private boolean m_side;
  private double m_startTime;
  private boolean m_stage;
  private double m_activeTime;
  private double m_activeSpeed;

  public CellBeltPulseSelect(CellTransportSubsystem transport, boolean leftSide, double firstTime, double firstSpeed,
      double secondTime, double secondSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_transport = transport;
    m_firstSpeed = firstSpeed;
    m_firstTime = firstTime;
    m_secondSpeed = secondSpeed;
    m_secondTime = secondTime;
    m_side = leftSide;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_stage = false;
    m_activeTime = m_firstTime;
    m_activeSpeed = m_firstSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() > m_startTime + m_activeTime) {
      m_stage = !m_stage;
      m_startTime = Timer.getFPGATimestamp();
      if (m_stage) {
        m_activeTime = m_secondTime;
        m_activeSpeed = m_secondSpeed;
      } else {
        m_activeTime = m_firstTime;
        m_activeSpeed = m_firstSpeed;
      }
    }

    if (m_side) {
      m_transport.runLeftBeltMotor(m_activeSpeed);
    } else {
      m_transport.runRightBeltMotor(m_activeSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_side)
      m_transport.runLeftBeltMotor(0);
    else
      m_transport.runRightBeltMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}