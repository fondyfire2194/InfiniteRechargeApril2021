/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogShooterSetup extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "ItemsLogged", "CameraDistance","RobotDistance", "BBHeight", "BBWidth", "TargetArea", "TiltAngle",
      "VertToTarget", "Tilt+Vert", "TestVOff", "TurretAngle", "HorToTarget", "Turret+Hor", "TargetHOff", "ReqdSpeed",
      "ActSpeed" };
  public static String[] units = { "Number", "Meters", "Meters", "Pixels", "Pixels", "SqPixels", "Degrees", "Degrees",
      "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "MPS", "MPS" };

  private int loopCtr;

  private final RevDrivetrain m_drive;
  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private double startDistance = 10;

  public LogShooterSetup(RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      RevShooterSubsystem shooter, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_limelight = limelight;
    m_turret = turret;
    m_tilt = tilt;
    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope2 = m_shooter.shootLogger.init("TestRun", "ShootSetup", names, units);
    SmartDashboard.putNumber("OPE2", ope2);
    loopCtr = 0;
    m_shooter.logSetupFileOpen = false;
    m_shooter.itemsLogged = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!m_shooter.logSetupFileOpen)
      loopCtr++;
    if (loopCtr > 500) {
      m_shooter.logSetupFileOpen = true;
      loopCtr = 0;
    }
    // log data every shot
    if (m_shooter.logSetupFileOpen)

      if (true) {

        m_shooter.itemsLogged++;
        m_shooter.shootLogger.writeData((double) m_shooter.itemsLogged, m_shooter.calculatedCameraDistance,
            startDistance - m_drive.getLeftDistance(), m_limelight.getBoundingBoxHeight(),
            m_limelight.getBoundingBoxWidth(), m_limelight.getTargetArea(), m_tilt.getAngle(),
            m_limelight.getdegVerticalToTarget(), m_tilt.getAngle() + m_limelight.getdegVerticalToTarget(),
            m_tilt.testVerticalOffset, m_turret.getAngle(), m_limelight.getdegRotationToTarget(),
            m_turret.getAngle() + m_limelight.getdegRotationToTarget(), m_turret.targetHorizontalOffset,
            m_shooter.requiredMps, m_shooter.getMPS(), m_shooter.getLeftAmps());
        
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd2 = m_shooter.shootLogger.close();
    m_shooter.endShootFile = false;
    m_shooter.logSetupFileOpen = false;
    SmartDashboard.putNumber("Close2", sd2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endShootFile;
  }
}
