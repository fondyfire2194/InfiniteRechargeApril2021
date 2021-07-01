// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;

public class PickupMove extends CommandBase {
  /** Creates a new PickupMove. */
  private final RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_speed;
  private double m_accelTime;
  private double currentSpeed;
  private double slowDownDistance = .4;
  private double slowDownRampTime = .1;
  private double upRampTime;
  private double slowDownRamp;
  private double upRamp;
  private double minSpeed = .25;
  private boolean accelerating;
  private boolean decelerating;
  private boolean plusDirection;
  private double remainingDistance;

  private double useSpeed;

  private int loopCtr;

  public PickupMove(RevDrivetrain drive, double endpoint, double speed, double accelTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_endpoint = endpoint;
    m_speed = Math.abs(speed);
    m_accelTime = accelTime;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    accelerating = true;
    decelerating = false;
    plusDirection = true;
    currentSpeed = minSpeed;
    upRampTime = m_accelTime;
    upRamp = (m_speed - minSpeed) / (upRampTime * 50);// per 20 ms
    slowDownRamp = m_speed / (slowDownRampTime * 50);
    remainingDistance = m_endpoint - m_drive.getLeftDistance();
    if (Math.abs(remainingDistance) < slowDownDistance) {
      slowDownDistance = 0;

    }
    if (DriverStation.getInstance().isOperatorControlEnabled()) {
      m_drive.logDriveItems = true;
    }

    m_drive.resetGyro();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * speed calculations are done in positive values and inverted based on
     * direction
     * 
     */

    remainingDistance = m_endpoint - m_drive.getLeftDistance();//

    if (remainingDistance < 0) {
      plusDirection = false;
    }

    if (currentSpeed >= m_speed) {
      accelerating = false;
    }

    if (accelerating) {
      currentSpeed += upRamp;
    }

    decelerating = Math.abs(remainingDistance) < slowDownDistance || decelerating;

    if (decelerating) {
      accelerating = false;
      currentSpeed -= slowDownRamp;
      if (currentSpeed < minSpeed)
        currentSpeed = minSpeed;
    }
    useSpeed = currentSpeed;

    if (!plusDirection)
      useSpeed = -useSpeed;

    m_drive.arcadeDrive(useSpeed, -m_drive.getYaw() * Pref.getPref("dRStKp"));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentSpeed = 0;
    m_drive.arcadeDrive(0, 0);
    m_drive.logDriveItems = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return !accelerating && plusDirection && m_drive.getLeftDistance() > m_endpoint
        || !accelerating && !plusDirection && m_drive.getLeftDistance() < m_endpoint;

  }
}
