// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.LimeLight;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReturnTiltTurret extends ParallelCommandGroup {
  /** Creates a new ReturnTiltTurret. */
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private double m_turretPosition;
  private double m_tiltPosition;
  private LimeLight m_limelight;
  private boolean m_on;
  private RevShooterSubsystem m_shooter;
  private double m_mps;

  public ReturnTiltTurret(RevTurretSubsystem turret, double turretPosition, RevTiltSubsystem tilt, double tiltPosition,
      LimeLight limelight, boolean on, RevShooterSubsystem shooter, double mps) {
    m_turret = turret;
    m_tilt = tilt;
    m_limelight = limelight;
    m_turretPosition = turretPosition;
    m_tiltPosition = tiltPosition;
    m_mps = mps;
    m_on = on;
    m_shooter = shooter;
    // Add your commands in the adddCommands() call, e.g. tilt
    // addCommands(new FooCommand(), new BarCommand());
    /**
     * Returns the tilt and turret to a position
     * 
     */

    addCommands(new PositionTurret(m_turret, m_turretPosition), new PositionTilt(m_tilt, m_tiltPosition),
        new UseVision(m_limelight, m_on), new RunShooter(m_shooter));

  }
}
