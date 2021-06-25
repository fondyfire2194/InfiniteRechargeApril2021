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

public class LogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "TiltAngle", "VertToTarget", "TargetVOff", "DriverVOff", "TiltLockErr",
      "TurretAngle", "HorToTarget", "TargetHOff", "DriverHOff", "TurrLockPE", "ReqdSpeed", "ActSpeed", "Battery",
      "TurretInPos", "TiltInPos", "VertOK", "HorOK", "ShooterAtSpeed", "IsShooting", "ValidTargetSeen" };

  public static String[] units = { "Number", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees", "Degrees",
      "Degrees", "Degrees", "Degrees", "Degrees", "MPS", "MPS", "Volts", "T/F", "T/F", "T/F", "T/F", "T/F", "T/F",
      "T/F" };

  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;

  private double tiltOnTarget;
  private double turretOnTarget;
  private double vertOnTarget;
  private double horOnTarget;
  private double isShooting;
  private double validTargetSeen;
  private double shooterAtSpeed;

  public LogShootData(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevShooterSubsystem shooter,
      LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_limelight = limelight;
    m_turret = turret;
    m_tilt = tilt;
    m_shooter = shooter;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int ope1 = m_shooter.shootLogger.init("TestRun", "Shoot", names, units);
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
      if (m_tilt.atTargetAngle())
        tiltOnTarget = 1;
      else
        tiltOnTarget = 0;

      if (m_turret.atTargetAngle())
        turretOnTarget = 1;
      else
        turretOnTarget = 0;

      if (m_limelight.getHorOnTarget(1))
        horOnTarget = 1;
      else
        horOnTarget = 0;

      if (m_limelight.getVertOnTarget(1))
        vertOnTarget = 1;
      else
        vertOnTarget = 0;

      if (m_tilt.validTargetSeen)
        validTargetSeen = 1;
      else
        validTargetSeen = 0;

      if (m_shooter.atSpeed())
        shooterAtSpeed = 1;
      else
        shooterAtSpeed = 0;

      m_shooter.shootLogger.writeData((double) step, m_tilt.getAngle(), m_limelight.getdegVerticalToTarget(),
          m_tilt.targetVerticalOffset, m_tilt.driverVerticalOffsetDegrees, m_tilt.getLockPositionError(),
          m_turret.getAngle(), m_limelight.getdegRotationToTarget(), m_turret.targetHorizontalOffset,
          m_turret.driverHorizontalOffsetDegrees, m_turret.getLockPositionError(), m_shooter.requiredMps,
          m_shooter.getMPS(), m_shooter.getBatteryVoltage(), turretOnTarget, tiltOnTarget, horOnTarget, vertOnTarget,
          shooterAtSpeed, isShooting, validTargetSeen);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    int sd = m_shooter.shootLogger.close();
    m_shooter.endShootFile = false;
    SmartDashboard.putNumber("Close", sd);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.endShootFile;
  }
}
