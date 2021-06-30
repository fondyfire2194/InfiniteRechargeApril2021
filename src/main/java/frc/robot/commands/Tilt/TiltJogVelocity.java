// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTiltSubsystem;

public class TiltJogVelocity extends CommandBase {
  /** Creates a new TurretJJogVelocity. */

  private final RevTiltSubsystem m_tilt;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private double maxSpeed = 2000;

  public TiltJogVelocity(RevTiltSubsystem tilt, Supplier<Double> xaxisSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tilt = tilt;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    addRequirements(m_tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(m_xaxisSpeedSupplier.get()) < .05)
      m_tilt.moveManually(0);
    else
      m_tilt.runAtVelocity(m_xaxisSpeedSupplier.get() * maxSpeed);
    // SmartDashboard.putNumber("TUSP", m_xaxisSpeedSupplier.get() * maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tilt.stop();
    m_tilt.targetAngle = m_tilt.getAngle();
  }

  // Returns true when the command sh.joould end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
