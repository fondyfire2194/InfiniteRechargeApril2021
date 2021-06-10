/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CellTransport;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;

public class JogRightBelt extends CommandBase {
  /**
   * Creates a new StartShooter.
   */
  private CellTransportSubsystem m_transport;

  private final Supplier<Double> m_xaxisSpeedSupplier;

  public JogRightBelt(CellTransportSubsystem transport, Supplier<Double> xaxisSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_transport = transport;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    addRequirements(m_transport);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_xaxisSpeedSupplier.get()) < .1)
      m_transport.stopRightBeltMotor();
    else
      m_transport.runRightBeltMotor(m_xaxisSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transport.stopRightBeltMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
