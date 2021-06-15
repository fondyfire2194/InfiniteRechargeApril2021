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
import frc.robot.Robot;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "CameraDistance", "BBHeight", "BBWidth", "TargetArea", "RobotSpeed",
      "TiltAngle", "VertToTarget", "Tilt+Vert", "TargetVOff", "DriverVOff", "TurretAngle", "HorToTarget", "Turret+Hor",
      "TargetHOff", "DriverVOff", "ReqdSpeed", "ActSpeed" };
  public static String[] units = { "Number", "Meters", "Pixels", "Pixels", "SqPixels", "MPS", "Degrees", "Degrees",
      "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "MPS", "MPS" };

  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final RevDrivetrain m_drive;
  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;

  public LogShootData(RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
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
    int ope1 = m_shooter.simpleCSVLogger.init("TestRun", "Shoot", names, units);
    SmartDashboard.putNumber("OPE1", ope1);
    loopCtr = 0;
    fileOpenNow = false;
    step = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // allow i second for file to be opened
    if (!fileOpenNow)
      loopCtr++;
    if (loopCtr > 500) {
      fileOpenNow = true;
      loopCtr = 0;
    }
    // log data every shot
    if (fileOpenNow)
      loopCtr++;
    if (m_shooter.logTrigger && loopCtr >= 5) {
      loopCtr = 0;
      step++;
      m_shooter.shootLogger.writeData((double) step, m_shooter.calculatedCameraDistance,
          m_limelight.getBoundingBoxHeight(), m_limelight.getBoundingBoxWidth(), m_limelight.getTargetArea(),
          m_drive.getLeftRate(), m_tilt.getAngle(), m_limelight.getdegVerticalToTarget(),
          m_tilt.getAngle() + m_limelight.getdegVerticalToTarget(), m_tilt.targetVerticalOffset,
          m_tilt.driverVerticalOffset, m_turret.getAngle(), m_limelight.getdegRotationToTarget(),
          m_turret.getAngle() + m_limelight.getdegRotationToTarget(), m_turret.targetHorizontalOffset,
          m_turret.driverHorizontalOffset, m_shooter.requiredMps, m_shooter.getMPS(), m_shooter.getLeftAmps());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_shooter.simpleCSVLogger.close();
    m_shooter.endFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endFile;
  }
}
