// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretShift extends InstantCommand {
  private final RevTurretSubsystem m_turret;
  private final RevShooterSubsystem m_shooter;
  private double m_change;

  public TurretShift(RevTurretSubsystem turret, double change, RevShooterSubsystem shooter) {
    // Use addRequirements() here to decl are subsystem dependencies.
    m_turret = turret;
    m_shooter = shooter;
    m_change = change;
  }

  // Called when the command is initially scheduled.
  /**
   * The change distance is at the target A positive change means shoot more right
   * so turret angle needs to be more positive.
   * 
   * Need to calculate the angular turret change which is sin-1(change/distance)
   * 
   * 
   */
  @Override
  public void initialize() {

    double distance;
    double correction = 0;
    distance = m_shooter.calculatedCameraDistance;
    
    if (distance > 2 && distance < 15)
      correction = Math.toDegrees(Math.asin(m_change / distance));

    m_turret.targetHorizontalOffset += correction;
  }
}
