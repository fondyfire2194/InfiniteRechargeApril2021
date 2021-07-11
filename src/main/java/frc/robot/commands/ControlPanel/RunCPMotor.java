// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ControlPanel;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class RunCPMotor extends CommandBase {
  /** Creates a new RunCPMotor. */
  private final ControlPanelSubsystem m_cp;
  private final Supplier<Double> m_xaxisSpeedSupplier;

  public RunCPMotor(ControlPanelSubsystem cp, Supplier<Double> xaxisSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_cp = cp;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("CPCMD", m_xaxisSpeedSupplier.get());
    if (Math.abs(m_xaxisSpeedSupplier.get()) < .1)
      m_cp.stopWheelMotor();
    else
      m_cp.turnWheelMotor(m_xaxisSpeedSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cp.stopWheelMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
