// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoMode0;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.subsystems.CellTransportSubsystem;
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
    public int shootNumber;

    public AutoFactory(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
            CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight, Compressor compressor,
            int shootNumber) {
        m_turret = turret;
        m_tilt = tilt;
        m_shooter = shooter;
        m_robotDrive = drive;
        m_transport = transport;
        m_limelight = limelight;
        m_compressor = compressor;

    }

    // front of power port shoot
    public SequentialCommandGroup getAutonomousCommand0() {
        return new AutoMode0(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight, m_compressor,
                shootNumber);
    }

    // // front of power port move and shoot
    // public SequentialCommandGroup getAutonomousCommand1() {
    // return new AutoMode1(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // left of power port shoot
    // public SequentialCommandGroup getAutonomousCommand2() {
    // return new AutoMode2(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // left of power port move and shoot
    // public SequentialCommandGroup getAutonomousCommand3() {
    // return new AutoMode3(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // right of power port shoot
    // public SequentialCommandGroup getAutonomousCommand4() {
    // return new AutoMode0(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // right of power port move and shoot
    // public SequentialCommandGroup getAutonomousCommand5() {
    // return new AutoMode5(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // fron of trench pickup and shoot
    // public SequentialCommandGroup getAutonomousCommand6() {
    // return new AutoMode6(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // front of trench double pickup and shoot
    // public SequentialCommandGroup getAutonomousCommand7() {
    // return new AutoMode7(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // front of trench double pickup and shoot
    // public SequentialCommandGroup getAutonomousCommand8() {
    // return new AutoMode8(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive,
    // m_limelight, m_compressor);
    // }

    // // cross line
    public PositionRobot getAutonomousCommand1() {
        return new PositionRobot(m_robotDrive, -1.5);
    }
}
