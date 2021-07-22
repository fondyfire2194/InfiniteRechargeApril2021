// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimber extends CommandBase {
  /** Creates a new TurnClimberMotor. */

  private final ClimberSubsystem m_climber;

  private Supplier<Double> m_xaxisSpeedSupplier;

  public RunClimber(ClimberSubsystem climber, Supplier<Double> xaxisSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_climber.getArmRaised() && m_climber.getRatchetUnlocked()) {

      m_climber.runMotor(-m_xaxisSpeedSupplier.get());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
