// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;

public class StartIntake extends CommandBase {
  /** Creates a new RunIntake. */
  private final RearIntakeSubsystem m_rearIntake;
  private final CellTransportSubsystem m_transport;

  public StartIntake(RearIntakeSubsystem rearIntake, CellTransportSubsystem transport) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rearIntake = rearIntake;
    m_transport = transport;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rearIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer.getFPGATimestamp();
    Shuffleboard.selectTab("Intake");
    m_transport.moveCellArm(m_transport.cellArmHoldCell);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_rearIntake.runIntakeMotor(.75);
    m_rearIntake.lowerArm();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rearIntake.stopIntakeMotor();
    m_rearIntake.raiseArm();
    Shuffleboard.selectTab("Competition");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
