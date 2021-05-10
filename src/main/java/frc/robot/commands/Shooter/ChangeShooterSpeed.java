/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ChangeShooterSpeed extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private double m_rpmChange;

  public ChangeShooterSpeed(RevShooterSubsystem shooter, double rpmChange) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_rpmChange = rpmChange;
    // addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentSpeed = m_shooter.requiredSpeed;

    double newSpeed = currentSpeed += m_rpmChange;

    if (newSpeed > HoodedShooterConstants.MAX_SHOOTER_RPM)
      newSpeed = HoodedShooterConstants.MAX_SHOOTER_RPM;

    if (newSpeed < 0)
      newSpeed = 0;

    m_shooter.requiredSpeed = newSpeed;
    SmartDashboard.putNumber("LL1", m_shooter.requiredSpeed);
  //  m_shooter.spin
    m_shooter.useCameraSpeed = false;
    m_shooter.cameraSpeedBypassed = true;
  }
}
