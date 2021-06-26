// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.PowerPortVisionTest;
import frc.robot.commands.AutoCommands.RightStartRetractShoot;
import frc.robot.commands.AutoCommands.AutoMode33BallTrench;
import frc.robot.commands.AutoCommands.AutoMode3M3BallTrench;
import frc.robot.commands.AutoCommands.AutoMode4BallTrench;
import frc.robot.commands.AutoCommands.AutoModeCenterPowerPort;
import frc.robot.commands.AutoCommands.AutoModeShieldGen;
import frc.robot.commands.AutoCommands.AutoModeShieldGenTestVision;
import frc.robot.commands.AutoCommands.AutoModeTrench;
import frc.robot.commands.AutoCommands.AutoModeTrenchTestVision;
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
            RearIntakeSubsystem intake, int shootNumber) {
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
    public SequentialCommandGroup getAutonomousCommand1() {
        return new AutoModeCenterPowerPort(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                m_compressor);
        // return new PowerPortVisionTest(m_shooter, m_turret, m_tilt, m_limelight);
    }

    // right of center retract shoot
    public SequentialCommandGroup getAutonomousCommand2() {
        return new RightStartRetractShoot(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                m_compressor);
    }

    // front of trench or left of center move pickup and and shoot together
    public SequentialCommandGroup getAutonomousCommand3() {
        return new AutoModeTrench(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight, m_compressor,
                m_intake);
        // return new AutoModeTrenchTestVision(m_shooter, m_turret, m_tilt, m_transport,
        // m_robotDrive, m_limelight,
        // m_compressor, m_intake);
    }

    // front of trench or left of center move pickup and and shoot
    public SequentialCommandGroup getAutonomousCommand4() {
        return new AutoMode4BallTrench(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                m_compressor, m_intake);
    }

    public SequentialCommandGroup getAutonomousCommand5() {
        return new AutoMode33BallTrench(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                m_compressor, m_intake);
    }

    public SequentialCommandGroup getAutonomousCommand6() {
        return new AutoMode3M3BallTrench(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                m_compressor, m_intake);
    }

}
