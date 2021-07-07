// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;

public class PickupMoveVelocity extends CommandBase {
  /** Creates a new PickupMove. */
  private final RevDrivetrain m_drive;
  private double m_endpoint;
  private double m_speed;
  private double currentMPS;
  private double decelDistance;
  private double maxDecel = 5;// mps/s
  private boolean plusDirection;
  private double remainingDistance;
  private double decelTime;
  private double maxAccel = 5;// mps/s
  private double accelTime;
  private double accelIncrementper20ms;

  private double startTime;
  private boolean endIt;
  private double useMPS;

  private int loopCtr;

  private boolean accelDone;
  private boolean decelerating;
  private double minSpeed = .1;

  private boolean dontStart;

  private boolean currentDirection;

  public PickupMoveVelocity(RevDrivetrain drive, double endpoint, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_endpoint = endpoint;
    m_speed = Math.abs(speed);

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    accelTime = m_speed / maxAccel;

    accelIncrementper20ms = m_speed / (accelTime * 50);

    decelTime = m_speed / maxDecel; // ex 3.5/7 =.5 sec

    decelDistance = (m_speed * decelTime) / 2; // ex (3.5 * .5)/2 = .

    currentMPS = 0;

    SmartDashboard.putNumber("EndPt", m_endpoint);
    SmartDashboard.putNumber("Speed", m_speed);
    SmartDashboard.putNumber("DeceLTime", decelTime);

    SmartDashboard.putNumber("decdis", decelDistance);

    SmartDashboard.putNumber("AccTime", accelTime);

    SmartDashboard.putNumber("AccInc", accelIncrementper20ms);

    plusDirection = m_endpoint > m_drive.getLeftDistance();
    SmartDashboard.putBoolean("Dir", plusDirection);
    startTime = Timer.getFPGATimestamp();

    endIt = false;

    decelerating = false;

    accelDone = false;

    useMPS = 0;
    loopCtr = 0;

    dontStart = Math.abs(m_endpoint - m_drive.getLeftDistance()) < .2;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * speed calculations are done in positive values and inverted based on
     * direction
     * 
     */
    loopCtr++;
    currentDirection = m_endpoint > m_drive.getLeftDistance();
    SmartDashboard.putBoolean("Acc", accelDone);
    SmartDashboard.putBoolean("Dcc", decelerating);
    SmartDashboard.putNumber("CurrMPS", currentMPS);

    if (!dontStart && !accelDone && !decelerating && currentMPS < m_speed) {

      currentMPS += accelIncrementper20ms;

      SmartDashboard.putNumber("AccTime", Timer.getFPGATimestamp() - startTime);

    }

    if (!accelDone && currentMPS >= m_speed) {

      accelDone = true;

      currentMPS = m_speed;

    }

    remainingDistance = m_endpoint - m_drive.getLeftDistance();

    if (accelDone && !decelerating && Math.abs(remainingDistance) <= decelDistance) {

      decelerating = true;

      SmartDashboard.putNumber("TimeTo Dec", Timer.getFPGATimestamp() - startTime);

    }

    if (decelerating && !endIt) {

      currentMPS = (m_speed * Math.abs(remainingDistance)) / decelDistance;

      if (currentMPS < minSpeed)

        currentMPS = minSpeed;

    }

    if (currentMPS >= m_speed)
      currentMPS = m_speed;
    // if (currentMPS < minSpeed)
    // currentMPS = minSpeed;

    useMPS = currentMPS;

    if (!plusDirection) {

      useMPS = -useMPS;

    }

    SmartDashboard.putNumber("UseSp", useMPS);
    SmartDashboard.putNumber("LCTR", loopCtr);
    double yawCorrection = useMPS * Pref.getPref("dRStKp");

    m_drive.smartVelocityControlMetersPerSec(useMPS - yawCorrection, useMPS + yawCorrection);

    endIt = dontStart || currentDirection != plusDirection
        || loopCtr > 10 && Math.abs(remainingDistance) < .1 && (plusDirection && m_drive.getLeftDistance() > m_endpoint)
        || (!plusDirection && m_drive.getLeftDistance() < m_endpoint) || endIt;

    SmartDashboard.putBoolean("ENDIT", endIt);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentMPS = 0;
    endIt = false;
    useMPS = 0;
    m_drive.stop();
    SmartDashboard.putNumber("Move Time", Timer.getFPGATimestamp() - startTime);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return endIt;

  }
}
