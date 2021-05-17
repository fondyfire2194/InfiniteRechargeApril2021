// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class ProjectileEquations extends CommandBase {
  /** Creates a new ProjectileEquations. */

  private static RevShooterSubsystem m_shooter;
  private static RevTiltSubsystem m_tilt;
  private static double g = -9.08;// m/secsqd

  public ProjectileEquations(RevShooterSubsystem shooter, RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tilt = tilt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = 0;
    double ih = HoodedShooterConstants.TILT_HEIGHT;
    double h = Units.inchesToMeters(96);// target height
    double v_0 = m_shooter.getMPS();
    double inAngle = m_tilt.getAngle();

    double vx0 = v_0 * Math.cos(inAngle);
    double xt = v_0 * t;

    double vy0 = v_0 * Math.sin(inAngle);
    double yt = .5 * g * (t * t) + vy0 * t + ih;

    /**
     * peak height is reached when vertical velocity is 0
     * 
     * 0 = (vy0 squared) + 2 * g * d
     * 
     * so vy0 = sqrt(2*g*h);
     * 
     * 
     * 
     */

    double vymhd = Math.sqrt(2 * 9.08 * 2.44);//69

    




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
