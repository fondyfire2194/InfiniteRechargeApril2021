// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevTurretSubsystem;

public class TurretJog extends CommandBase {
  /** Creates a new TurretJog. */
  private final RevTurretSubsystem m_turret;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private XboxController m_controller;

  public TurretJog(RevTurretSubsystem turret, Supplier<Double> xaxisSpeedSupplier, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_controller = controller;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double temp = 0;
    boolean pressed = m_controller.getRawButton(6);
    if (Math.abs(m_xaxisSpeedSupplier.get()) < .1)
      m_turret.moveManually(0);
    else
      temp = m_xaxisSpeedSupplier.get();
    m_turret.moveManually(temp);
    if (pressed)
      temp = temp / 2;
    m_turret.enableSofLimits(!pressed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
    m_turret.targetAngle = m_turret.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
