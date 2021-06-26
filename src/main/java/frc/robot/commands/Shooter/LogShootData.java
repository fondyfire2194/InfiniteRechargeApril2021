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
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class LogShootData extends CommandBase {
  /**
   * Creates a new LogDistanceData.
   */
  public final String[] names = { "Step", "VertToTarget", "HorToTarget", "Battery", "ShooterAtSpeed", "IsShooting",
      "ArmPosition", "ValidTargetSeen" };

  public static String[] units = { "Number", "Degrees", "Degrees", "Volts", "T/F", "T/F", "PWM", "T/F" };

  private int loopCtr;
  private boolean fileOpenNow;
  private int step;
  private final LimeLight m_limelight;
  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private final CellTransportSubsystem m_transport;

  private double isShooting;
  private double validTargetSeen;
  private double shooterAtSpeed;

  public LogShootData(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevShooterSubsystem shooter,
      CellTransportSubsystem transport, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_limelight = limelight;
    m_turret = turret;
    m_tilt = tilt;
    m_shooter = shooter;
    m_transport = transport;
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

      if (m_tilt.validTargetSeen)
        validTargetSeen = 1;
      else
        validTargetSeen = 0;

      if (m_shooter.atSpeed())
        shooterAtSpeed = 1;
      else
        shooterAtSpeed = 0;

      m_shooter.shootLogger.writeData((double) step, m_limelight.getdegVerticalToTarget(),

          m_limelight.getdegRotationToTarget(), m_shooter.getBatteryVoltage(), shooterAtSpeed, isShooting,
          m_transport.getArmAngle(), validTargetSeen);
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
