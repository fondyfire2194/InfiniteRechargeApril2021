// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;

public class LimelightSetPipeline extends CommandBase {
  /** Creates a new LimelightLeds. */
  private final LimeLight m_limelight;
  private int m_number;
  private double m_startTime;

  public LimelightSetPipeline(LimeLight limelight, int number) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    m_number = number;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_number > 9)
      m_number = 9;
    if (m_number < 0)
      m_number = 0;

    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setPipeline(m_number);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() > m_startTime + .1;
  }
}
