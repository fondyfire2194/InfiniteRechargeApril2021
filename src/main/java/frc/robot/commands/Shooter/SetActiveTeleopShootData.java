// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand; 
import frc.robot.ShootData;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetActiveTeleopShootData extends InstantCommand {

  private int m_value;

  public SetActiveTeleopShootData(int value) {
    m_value = value;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_value) {
      case 0:
      ShootData.activeTeleopShootSpeed = ShootData.centerPowerPortConstants.shootSpeed;
      ShootData.activeTeleopTurretAngle = ShootData.centerPowerPortConstants.turretAngle;
      ShootData.activeTeleopTurretOffset = ShootData.centerPowerPortConstants.turretOffset;
      ShootData.activeTeleopTiltAngle = ShootData.centerPowerPortConstants.tiltAngle;
      ShootData.activeTeleopTiltOffset = ShootData.centerPowerPortConstants.tiltOffset;
      break;
  
      case 1:
        ShootData.activeTeleopShootSpeed = ShootData.shieldGenConstants.shootSpeed1;
        ShootData.activeTeleopTurretAngle = ShootData.shieldGenConstants.turretAngle1;
        ShootData.activeTeleopTurretOffset = ShootData.shieldGenConstants.turretOffset1;
        ShootData.activeTeleopTiltAngle = ShootData.shieldGenConstants.tiltAngle1;
        ShootData.activeTeleopTiltOffset = ShootData.shieldGenConstants.tiltOffset1;
        break;

        case 2:
        ShootData.activeTeleopShootSpeed = ShootData.trenchShotConstants.shootSpeed1;
        ShootData.activeTeleopTurretAngle = ShootData.trenchShotConstants.turretAngle1;
        ShootData.activeTeleopTurretOffset = ShootData.trenchShotConstants.turretOffset1;
        ShootData.activeTeleopTiltAngle = ShootData.trenchShotConstants.tiltAngle1;
        ShootData.activeTeleopTiltOffset = ShootData.trenchShotConstants.tiltOffset1;
        break;

  

      default:
        break;

    }
  }
}
