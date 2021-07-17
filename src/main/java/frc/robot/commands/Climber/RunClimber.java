// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RunClimber extends CommandBase {
  /** Creates a new TurnClimberMotor. */

  private final ClimberSubsystem m_climber;
  private double m_speed;
  private boolean m_direction;
  private XboxController m_gamepad;

  public RunClimber(ClimberSubsystem climber, XboxController gamepad, boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_direction = direction;
    m_gamepad = gamepad;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speed = m_gamepad.getTriggerAxis(Hand.kRight) * .75;

    if (!m_direction)
    
      m_speed = -m_speed;

    if (m_climber.getArmRaised() && m_climber.getRatchetUnlocked()) {

      m_climber.runMotor(m_speed);
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
