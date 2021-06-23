// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ShootData;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetActiveTeleopShootData extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private int m_value;

  public SetActiveTeleopShootData(RevShooterSubsystem shooter, int value) {
    m_value = value;
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.teleopSetupIndex = m_value;
    switch (m_value) {
      case 0:
        SmartDashboard.putBoolean("GOTHER", true);

        ShootData.activeTeleopShootSpeed = ShootData.centerPowerPortConstants.shootSpeed;
        ShootData.activeTeleopTurretAngle = ShootData.centerPowerPortConstants.turretAngle;
        ShootData.activeTeleopTurretOffset = ShootData.centerPowerPortConstants.turretOffset;
        ShootData.activeTeleopTiltAngle = ShootData.centerPowerPortConstants.tiltAngle;
        ShootData.activeTeleopTiltOffset = ShootData.centerPowerPortConstants.tiltOffset;
        SmartDashboard.putNumber("SDATS", ShootData.centerPowerPortConstants.tiltAngle);
        SmartDashboard.putNumber("SDCPP", ShootData.activeTeleopTiltAngle);

        break;

      case 1:
        ShootData.activeTeleopShootSpeed = ShootData.shieldGenConstants.shootSpeed;
        ShootData.activeTeleopTurretAngle = ShootData.shieldGenConstants.turretAngle;
        ShootData.activeTeleopTurretOffset = ShootData.shieldGenConstants.turretOffset;
        ShootData.activeTeleopTiltAngle = ShootData.shieldGenConstants.tiltAngle;
        ShootData.activeTeleopTiltOffset = ShootData.shieldGenConstants.tiltOffset;
        break;

      case 2:
        ShootData.activeTeleopShootSpeed = ShootData.trenchShotConstants.shootSpeed;
        ShootData.activeTeleopTurretAngle = ShootData.trenchShotConstants.turretAngle;
        ShootData.activeTeleopTurretOffset = ShootData.trenchShotConstants.turretOffset;
        ShootData.activeTeleopTiltAngle = ShootData.trenchShotConstants.tiltAngle;
        ShootData.activeTeleopTiltOffset = ShootData.trenchShotConstants.tiltOffset;
        break;

      case 3:
        ShootData.activeTeleopShootSpeed = ShootData.behindControlPanelShotConstants.shootSpeed;
        ShootData.activeTeleopTurretAngle = ShootData.behindControlPanelShotConstants.turretAngle;
        ShootData.activeTeleopTurretOffset = ShootData.behindControlPanelShotConstants.turretOffset;
        ShootData.activeTeleopTiltAngle = ShootData.behindControlPanelShotConstants.tiltAngle;
        ShootData.activeTeleopTiltOffset = ShootData.behindControlPanelShotConstants.tiltOffset;
        break;

      case 4:

        ShootData.activeTeleopShootSpeed = ShootData.lowShotConstants.shootSpeed;
        ShootData.activeTeleopTurretAngle = ShootData.lowShotConstants.turretAngle;
        ShootData.activeTeleopTurretAngle = ShootData.lowShotConstants.tiltAngle;
        break;
      default:
        break;

       

    }
  }
}
