// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.AutoCommands.PowerPort.AutoCenterShootMove;
import frc.robot.commands.AutoCommands.ShieldGenOne.ShieldGenAuto1;
import frc.robot.commands.AutoCommands.ShieldGenTwo.ShieldGenAuto2;
import frc.robot.commands.AutoCommands.TrenchOne.TrenchAuto1;
import frc.robot.commands.AutoCommands.TrenchThree.TrenchAuto3;
import frc.robot.commands.AutoCommands.TrenchTwo.TrenchAuto2;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class AutoFactory {

  private final RevTurretSubsystem m_turret;
  private final RevTiltSubsystem m_tilt;
  private final RevShooterSubsystem m_shooter;
  private final RevDrivetrain m_robotDrive;
  private final CellTransportSubsystem m_transport;
  private final LimeLight m_limelight;
  private final Compressor m_compressor;
  private final RearIntakeSubsystem m_intake;
  public int shootNumber;

  public AutoFactory(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight, Compressor compressor,
      RearIntakeSubsystem intake) {
    m_turret = turret;
    m_tilt = tilt;
    m_shooter = shooter;
    m_robotDrive = drive;
    m_transport = transport;
    m_limelight = limelight;
    m_compressor = compressor;
    m_intake = intake;
  }

  // front of power port move and shoot
  public ParallelRaceGroup getAutonomousCommand1() {

    return new AutoCenterShootMove(m_shooter, m_robotDrive, m_turret, m_tilt, m_limelight, m_compressor, m_transport, m_intake);
    // right of center line retract shoot
  }

  // front of trench or shoot 3 move pickup shoot 1
  public ParallelRaceGroup getAutonomousCommand2() {
    return new TrenchAuto1(m_shooter, m_robotDrive, m_tilt, m_turret, m_transport, m_intake, m_limelight, m_compressor);
  }

  // front of trench or shoot 3 move pickup shoot 2
  public ParallelRaceGroup getAutonomousCommand3() {
    return new TrenchAuto2(m_shooter, m_robotDrive, m_tilt, m_turret, m_transport, m_intake, m_limelight, m_compressor);
  }

  // front of trench or shoot 3 move pickup shoot 3
  public ParallelRaceGroup getAutonomousCommand4() {
    return new TrenchAuto3(m_shooter, m_robotDrive, m_tilt, m_turret, m_transport, m_intake, m_limelight, m_compressor);
  }

  // center Line or shoot 3 move pickup shoot 1
  public ParallelRaceGroup getAutonomousCommand5() {
    return new ShieldGenAuto1(m_shooter, m_robotDrive, m_tilt, m_turret, m_transport, m_intake, m_limelight,
        m_compressor);
  }

  // front of trench or shoot 3 move pickup shoot 3
  public ParallelRaceGroup getAutonomousCommand6() {
    return new ShieldGenAuto2(m_shooter, m_robotDrive, m_tilt, m_turret, m_transport, m_intake, m_limelight,
        m_compressor);
  }

}
