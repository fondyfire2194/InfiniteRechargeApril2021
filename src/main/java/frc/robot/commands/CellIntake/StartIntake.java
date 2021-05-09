// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CellIntake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.Constants.IntakeConstants;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.subsystems.RearIntakeSubsystem;

public class StartIntake extends CommandBase {
  /** Creates a new RunIntake. */
  private final RearIntakeSubsystem m_rearIntake;
  private final LimeLight m_limelight;

  public StartIntake(RearIntakeSubsystem rearIntake, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_rearIntake = rearIntake;
    m_limelight= limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rearIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer.getFPGATimestamp();
    m_limelight.setStream((StreamType.kPiPSecondary));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_rearIntake.runIntakeMotor(IntakeConstants.REAR_SPEED);
    m_rearIntake.lowerArm();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_rearIntake.runIntakeMotor(0);
    m_rearIntake.raiseArm();
    m_limelight.setStream(StreamType.kStandard);


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
