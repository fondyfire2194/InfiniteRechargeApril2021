// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevElevatorSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class ElevJog extends CommandBase {
  /** Creates a new TurretJog. */
  private final RevElevatorSubsystem m_elev;
  private double m_speed;

  public ElevJog(RevElevatorSubsystem elev, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elev = elev;
    m_speed = speed;
    addRequirements(m_elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elev.moveManually(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elev.moveManually(0);
  }

  // Returns true when the command sh.joould end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
