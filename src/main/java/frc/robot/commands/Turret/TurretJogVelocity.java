// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class TurretJogVelocity extends CommandBase {
  /** Creates a new TurretJJogVelocity. */

  private final RevTurretSubsystem m_turret;
  private XboxController m_controller;

  private final Supplier<Double> m_xaxisSpeedSupplier;
  private double maxSpeed = 250;

  public TurretJogVelocity(RevTurretSubsystem turret, Supplier<Double> xaxisSpeedSupplier, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_controller = controller;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(m_xaxisSpeedSupplier.get()) < .05)
      m_turret.moveManually(0);
    else
      m_turret.runAtVelocity(m_xaxisSpeedSupplier.get() * maxSpeed);
     SmartDashboard.putNumber("TUSP", m_xaxisSpeedSupplier.get() * maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command sh.joould end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
