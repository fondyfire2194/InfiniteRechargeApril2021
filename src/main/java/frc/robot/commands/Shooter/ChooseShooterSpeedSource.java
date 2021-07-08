// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import org.ejml.dense.row.mult.MatrixMatrixMult_CDRM;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChooseShooterSpeedSource extends InstantCommand {
  private RevShooterSubsystem m_shooter;
  private RevTiltSubsystem m_tilt;
  private RevTurretSubsystem m_turret;
  private int m_choice;

  public ChooseShooterSpeedSource(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret,
      int choice) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tilt = tilt;
    m_turret = turret;
    m_choice = choice;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.activeSpeedSource = m_shooter.speedSource[m_choice];
    switch (m_choice) {

      case 0: {
        m_shooter.useProgramSpeed = true;
        m_shooter.useSetupSlider = false;
        m_shooter.useCameraSpeed = false;
        m_shooter.useDriverSpeed = false;

        m_tilt.useSetupVertOffset = false;
        m_turret.useSetupHorOffset = false;
        m_tilt.cameraCalculatedTiltOffset = 0;

      }
        break;

      case 1: {
        m_shooter.useProgramSpeed = false;
        m_shooter.useSetupSlider = false;
        m_shooter.useCameraSpeed = true;
        m_shooter.useDriverSpeed = false;

        m_tilt.useSetupVertOffset = false;
        m_turret.useSetupHorOffset = false;
      }

        break;
      case 2: {
        m_shooter.useProgramSpeed = false;
        m_shooter.useSetupSlider = false;
        m_shooter.useCameraSpeed = false;
        m_shooter.useDriverSpeed = true;
        m_tilt.useSetupVertOffset = false;
        m_turret.useSetupHorOffset = false;
        m_tilt.cameraCalculatedTiltOffset = 0;
      }

        break;
      case 3: {
        m_shooter.useProgramSpeed = false;
        m_shooter.useSetupSlider = true;
        m_shooter.useCameraSpeed = false;
        m_shooter.useDriverSpeed = false;
        m_tilt.useSetupVertOffset = true;
        m_turret.useSetupHorOffset = true;
        m_tilt.cameraCalculatedTiltOffset = 0;
      }

        break;
      default:
        break;

    }
  }
}