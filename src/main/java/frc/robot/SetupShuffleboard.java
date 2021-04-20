// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.ControlPanel.ControlPanelArm;
import frc.robot.commands.ControlPanel.PositionNumberRevs;
import frc.robot.commands.ControlPanel.PositionToColor;
import frc.robot.commands.ControlPanel.ToggleLookForColor;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.ResetPose;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.RobotDrive.TurnToAngleProfiled;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.DecreaseShooterSpeed;
import frc.robot.commands.Shooter.IncreaseShooterSpeed;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Shooter.StopShooterWheels;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.ResetTiltAngle;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Turret.StopTurret;
import frc.robot.commands.Vision.LimelightCamMode;
import frc.robot.commands.Vision.LimelightLeds;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.LimelightStreamMode;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/** Add your docs here. */
public class SetupShuffleboard {
        private final RevTiltSubsystem m_tilt;
        private final RevTurretSubsystem m_turret;
        private final RevDrivetrain m_robotDrive;
        private final RevShooterSubsystem m_shooter;
        private final CellTransportSubsystem m_transport;
        private final Compressor m_compressor;
        private final LimeLight m_limelight;
        private final ControlPanelSubsystem m_controlPanel;
        private final RearIntakeSubsystem m_intake;
        private final FondyFireTrajectory m_traj;
        private final ClimberSubsystem m_climber;
        private boolean m_showTurret = true;
        private boolean m_showTilt = true;
        private boolean m_showShooter = true;
        private boolean m_showRobot = true;
        private boolean m_showClimber = true;
        private boolean m_showControlPanel = true;
        private boolean m_showTransport = true;
        private boolean m_showVision = true;
        private boolean m_showTrajectory = true;
        private boolean m_showSubsystems = true;
        private HttpCamera LLFeed;
        public double timeToStart;

        public SendableChooser<Integer> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();

        public SetupShuffleboard(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CellTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, ControlPanelSubsystem panel, RearIntakeSubsystem intake,
                        FondyFireTrajectory traj, ClimberSubsystem climber) {
                m_turret = turret;
                m_tilt = tilt;
                m_robotDrive = drive;
                m_transport = transport;
                m_compressor = compressor;
                m_shooter = shooter;
                m_limelight = limelight;
                m_controlPanel = panel;
                m_intake = intake;
                m_traj = traj;
                m_climber = climber;

                /**
                 * 
                 * Pre round
                 */

                // Put
                // autonomous chooser on the dashboard.
                // The first argument is the root container
                // The second argument is whether logging and config should be given separate
                // tabs
                Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1) // make the widget
                                // 2x1
                                .withPosition(0, 0); // place it in the top-left corner

                int place = 0;
                autoChooser.setDefaultOption("Center Start Retract Shoot", place);
                place = 1;
                autoChooser.addOption("Left Start Retract Shoot", place);
                place = 2;
                autoChooser.addOption("Right Start Retract Shoot", place);
                place = 3;
                autoChooser.addOption("Right Shoot Trench Pickup", place);
                place = 4;
                autoChooser.addOption("Cross Line", place);
                place = 5;
                autoChooser.addOption(("Example S Shape"), place);
                place = 6;
                autoChooser.addOption(("AutoNH1"), place);
                place = 7;
                autoChooser.addOption(("Trench Start"), place);

                Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1) // make the widget
                                // 2x1
                                .withPosition(2, 0); //

                startDelayChooser.setDefaultOption("No Delay", 0);
                startDelayChooser.addOption("One Second", 1);
                startDelayChooser.addOption("Two Seconds", 2);
                startDelayChooser.addOption("Three Seconds", 3);
                startDelayChooser.addOption("four Seconds", 4);
                startDelayChooser.addOption("Five Seconds", 5);

                ShuffleboardLayout preMatch = Shuffleboard.getTab("Pre-Round").getLayout("Info", BuiltInLayouts.kList)
                                .withPosition(0, 1).withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                preMatch.addNumber("TiltView", () -> m_tilt.getAngle()).withWidget(BuiltInWidgets.kDial)
                                .withProperties(Map.of("Min", -2, "Max", 70, "Show Text", true)).withSize(2, 2);

                preMatch.addNumber("TurretView", () -> m_turret.getAngle()).withWidget(BuiltInWidgets.kNumberBar)
                                .withProperties(Map.of("Min", -120, "Max", 120, "Show Text", true)).withSize(2, 1);

                preMatch.add("Tilt Down OK", m_tilt.m_reverseLimit.get()).withWidget(BuiltInWidgets.kBooleanBox)
                                .withSize(2, 1).withPosition(0, 4);

                if (RobotBase.isReal()) {

                        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

                        Shuffleboard.getTab("Pre-Round").add("Limelight", LLFeed)
                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 0).withSize(3, 2)
                                        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//
                }

                /**
                 * 
                 * Competition Driver Tab
                 * 
                 */

                ShuffleboardLayout competition = Shuffleboard.getTab("Competition")
                                .getLayout("Info", BuiltInLayouts.kList).withPosition(0, 1).withSize(1, 4)
                                .withProperties(Map.of("Label position", "TOP"));

                competition.addNumber("TiltView", () -> m_tilt.getAngle()).withWidget(BuiltInWidgets.kDial)
                                .withProperties(Map.of("Min", -2, "Max", 70, "Show Text", true)).withSize(2, 2);

                competition.addNumber("TurretView", () -> m_turret.getAngle()).withWidget(BuiltInWidgets.kNumberBar)
                                .withProperties(Map.of("Min", -120, "Max", 120, "Show Text", true)).withSize(2, 1);

                competition.addNumber("MatchTime", () -> DriverStation.getInstance().getMatchTime());

                ShuffleboardLayout active = Shuffleboard.getTab("Competition").getLayout("Active", BuiltInLayouts.kList)
                                .withPosition(1, 1).withSize(1, 5).withProperties(Map.of("Label position", "TOP"));
                active.addNumber("Pipeline", () -> m_limelight.getPipeline());
                active.addNumber("ShooterSpeed", () -> m_shooter.getRPM());
                active.addNumber("Tilt", () -> m_tilt.targetAngle);
                active.addNumber("Turret", () -> m_turret.targetAngle);

                if (RobotBase.isReal()) {

                        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
                        ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("Competition");
                        driverDisplayTab.add("Limelight", LLFeed).withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(3, 0).withSize(6, 5)
                                        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify
                                                                                                                // widget
                                                                                                                // properties
                                                                                                                // here
                }

                /**
                 * 
                 * Shooter Turret
                 * 
                 */
                if (m_showTurret) {
                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurretTilt")
                                        .getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                           // commands

                        turretCommands.add("Reset to 0", new ResetTurretAngle(m_turret));
                        turretCommands.add("Position To 0", new PositionTurret(m_turret, 0));// degrees
                        turretCommands.add("Position To 20", new PositionTurret(m_turret, 20));// degrees
                        turretCommands.add("Position To 50", new PositionTurret(m_turret, 50));
                        turretCommands.add("50 to Vision", new PositionTurretToVision(m_turret, m_limelight, 50));
                        turretCommands.add("StopTurret", new StopTurret(m_turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(m_turret));

                        ShuffleboardLayout turretValues = Shuffleboard.getTab("SetupTurretTilt")
                                        .getLayout("TurretValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        turretValues.addNumber("TUAngle", () -> m_turret.getAngle());
                        turretValues.addNumber("TUTgt", () -> m_turret.targetAngle);
                        turretValues.addNumber("Pct", () -> m_turret.getOut());
                        turretValues.addNumber("Amps", () -> m_turret.getAmps());
                        turretValues.addNumber("Speed", () -> m_turret.getSpeed());
                        turretValues.addNumber("Faults", () -> m_turret.getFaults());
                        turretValues.addBoolean("PlusLimit", () -> m_turret.onPlusSoftwareLimit())
                                        .withWidget(BuiltInWidgets.kTextView);
                        turretValues.addBoolean("MinusLimit", () -> m_turret.onMinusSoftwareLimit())
                                        .withWidget(BuiltInWidgets.kTextView);
                        turretValues.addBoolean("SWLimitEn", () -> m_turret.getSoftwareLimitsEnabled())
                                        .withWidget(BuiltInWidgets.kTextView);
                        turretValues.addBoolean("InPosition", () -> m_turret.atTargetAngle())
                                        .withWidget(BuiltInWidgets.kTextView);
                        turretValues.add("Cmd", m_turret);
                }
                /**
                 * 
                 * Shooter Tilt
                 * 
                 */
                if (m_showTilt) {
                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTurretTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Reset To 0", new ResetTiltAngle(m_tilt));
                        tiltCommands.add("Position To 5", new PositionTilt(m_tilt, 5));
                        tiltCommands.add("Position To 10", new PositionTilt(m_tilt, 10));

                        tiltCommands.add("To Bottom Switch", new TiltMoveToReverseLimit(m_tilt));
                        tiltCommands.add("5 to Vision", new PositionTiltToVision(m_tilt, m_limelight, 5));
                        tiltCommands.add("StopTilt", new StopTilt(m_tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(m_tilt));

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTurretTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TIAngle", () -> m_tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> m_tilt.targetAngle);
                        tiltValues.addNumber("PCT", () -> m_tilt.getOut());
                        tiltValues.addNumber("Amps", () -> m_tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> m_tilt.getSpeed());
                        tiltValues.addNumber("Faults", () -> m_tilt.getFaults());
                        tiltValues.addBoolean("PlusSWLimit", () -> m_tilt.onPlusSoftwareLimit())
                                        .withWidget(BuiltInWidgets.kTextView);
                        tiltValues.addBoolean("MinusSWLimit", () -> m_tilt.onMinusSoftwareLimit())
                                        .withWidget(BuiltInWidgets.kTextView);
                        tiltValues.addBoolean("InPosition", () -> m_tilt.atTargetAngle())
                                        .withWidget(BuiltInWidgets.kTextView);
                        tiltValues.addBoolean("OnBottomLS", () -> m_tilt.m_reverseLimit.get())
                                        .withWidget(BuiltInWidgets.kTextView);

                        tiltValues.add("Cmd", m_tilt);
                }
                /**
                 * 
                 * Shooter and Transport
                 * 
                 */
                if (m_showShooter) {
                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("Shooter", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                           // commands

                        shooterCommands.add("Shooter", new StartShooterWheels(m_shooter, 500));
                        shooterCommands.add("Stop", new StopShooterWheels(m_shooter));
                        shooterCommands.add("Stop Shoot", new StopShoot(m_shooter, m_transport));
                        shooterCommands.add("Inc 10% ", new IncreaseShooterSpeed(m_shooter));
                        shooterCommands.add("Dec 10% ", new DecreaseShooterSpeed(m_shooter));
                        shooterCommands.add("Shoot", new ShootCells(m_shooter, m_transport, m_compressor, 3000, 0));
                        shooterCommands.add("ClearFaults", new ClearShFaults(m_shooter));

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(3, 5).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        shooterValues.addNumber("LeftRPM", () -> m_shooter.getRPM());
                        shooterValues.addNumber("LeftAmps", () -> m_shooter.getLeftAmps());
                        shooterValues.addNumber("RightAmps", () -> m_shooter.getRightAmps());
                        shooterValues.addNumber("SpeedCommand", () -> m_shooter.requiredSpeed);
                        shooterValues.addNumber("Faults", () -> m_shooter.getFaults());
                        shooterValues.addBoolean("AtSpeed", () -> m_shooter.atSpeed())
                                        .withWidget(BuiltInWidgets.kTextView);
                        shooterValues.add(m_shooter);
                }

                if (m_showTransport) {

                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(5, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        transportValues.addNumber("LeftBeltAmps", () -> m_transport.getLeftBeltMotorAmps());
                        transportValues.addNumber("RightBeltAmps", () -> m_transport.getRightBeltMotorAmps());
                        transportValues.addNumber("FrontRollerAmps", () -> m_transport.getFrontRollerMotorAmps());
                        transportValues.addNumber("RearRollerAmps", () -> m_transport.getRearRollerMotorAmps());
                        transportValues.addNumber("LeftBeltOut", () -> m_transport.getLeftBelt());
                        transportValues.addNumber("RighBeltOut", () -> m_transport.getRightBelt());
                        transportValues.addNumber("FrontRollerOut", () -> m_transport.getFrontRoller());
                        transportValues.addNumber("RearRollerOut", () -> m_transport.getRearRoller());

                        transportValues.add("Cmd", m_transport);

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(7, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeValues.addNumber("Motor Amps", () -> m_intake.getMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> m_intake.getMotor());
                        intakeValues.add("ITK", m_intake);
                }
                /**
                 * 
                 * Robot
                 * 
                 */
                if (m_showRobot) {
                        ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        robotCommands.add("Reset Enc", new ResetEncoders(m_robotDrive));
                        robotCommands.add("Reset Gyro", new ResetGyro(m_robotDrive));
                        robotCommands.add("Reset Pose", new ResetPose(m_robotDrive));
                        robotCommands.add("Pos to 3M", new PositionRobot(m_robotDrive, 3.));
                        robotCommands.add("Pos to 0M", new PositionRobot(m_robotDrive, 0));
                        robotCommands.add("Rot to 90", new TurnToAngleProfiled(m_robotDrive, 90));
                        robotCommands.add("Rot to 0", new TurnToAngleProfiled(m_robotDrive, 0));
                        robotCommands.add("Rot to -90", new TurnToAngleProfiled(m_robotDrive, -90));
                        robotCommands.add("ClearFaults", new ClearRobFaults(m_robotDrive));
                        robotCommands.add("Stop Robot", new StopRobot(m_robotDrive));

                        ShuffleboardLayout robotValues = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("RobotValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        robotValues.addNumber("LeftMeters", () -> m_robotDrive.getLeftDistance());
                        robotValues.addNumber("RightMeters", () -> m_robotDrive.getRightDistance());
                        robotValues.addNumber("LeftVelMPS", () -> m_robotDrive.getLeftRate());
                        robotValues.addNumber("RightVelMPS", () -> m_robotDrive.getRightRate());
                        robotValues.addNumber("LeftOut", () -> m_robotDrive.getLeftOut());
                        robotValues.addNumber("RightOut", () -> m_robotDrive.getRightOut());
                        robotValues.addNumber("LeftAmps", () -> m_robotDrive.getLeftAmps());
                        robotValues.addNumber("RightAmps", () -> m_robotDrive.getRightAmps());
                        robotValues.addNumber("Gyro Yaw", () -> m_robotDrive.getYaw());
                        robotValues.addNumber("Faults", () -> m_robotDrive.getFaults());
                        robotValues.add("Cmd", m_robotDrive);

                }
                /**
                 * 
                 * Run Trajectory
                 * 
                 */
                if (m_showTrajectory) {
                        SendableChooser<Trajectory> trajChooser = new SendableChooser<>();
                        Shuffleboard.getTab("SetupRobot").add("Trajectories", trajChooser).withSize(2, 1)
                                        .withPosition(6, 0);
                        trajChooser.setDefaultOption("TrenchStart", m_traj.trenchStartOne);
                        trajChooser.addOption("CenterStart", m_traj.centerStart);
                        trajChooser.addOption("Example", m_traj.example);

                        Shuffleboard.getTab("SetupRobot")
                                        .add("StartTraj",
                                                        m_traj.getRamsete(trajChooser.getSelected())
                                                                        .andThen(() -> drive.tankDriveVolts(0, 0)))
                                        .withPosition(6, 1).withSize(2, 1);

                        /**
                         * 
                         * Subsystems
                         * 
                         */
                }

                if (m_showSubsystems) {
                        ShuffleboardLayout subSystems = Shuffleboard.getTab("Subsystems")
                                        .getLayout("All", BuiltInLayouts.kList).withPosition(0, 0).withSize(3, 7)
                                        .withProperties(Map.of("Label position", "LEFT")); //
                                                                                           // labels
                                                                                           // for
                        subSystems.add("Drive", m_robotDrive);
                        subSystems.add("Shooter", m_shooter);
                        subSystems.add("Turret", m_turret);
                        subSystems.add("Tilt", m_tilt);
                        subSystems.add("Transport", m_transport);
                        subSystems.add("Control Panel", m_controlPanel);
                        subSystems.add("Intake", m_intake);
                        subSystems.add("Climber", m_climber);
                }
                /**
                 * 
                 * Vision
                 * 
                 */
                if (m_showVision) {
                        ShuffleboardLayout zoomCommands = Shuffleboard.getTab("Vision")
                                        .getLayout("Zoom", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP")); //
                                                                                          // labels
                                                                                          // for

                        zoomCommands.add("No Zoom", new LimelightSetPipeline(m_limelight, 0));
                        zoomCommands.add("2XZoom", new LimelightSetPipeline(m_limelight, 1));
                        zoomCommands.add("3X Zoom", new LimelightSetPipeline(m_limelight, 2));

                        ShuffleboardLayout cameraCommands = Shuffleboard.getTab("Vision")
                                        .getLayout("Camera", BuiltInLayouts.kList).withPosition(1, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP")); // labels for

                        cameraCommands.add("DriverCam", new LimelightCamMode(m_limelight, CamMode.kdriver));
                        cameraCommands.add("VisionCam", new LimelightCamMode(m_limelight, CamMode.kvision));

                        cameraCommands.add("SideBySideStream",
                                        new LimelightStreamMode(m_limelight, StreamType.kStandard));
                        cameraCommands.add("MainPIP", new LimelightStreamMode(m_limelight, StreamType.kPiPMain));
                        cameraCommands.add("SecIP", new LimelightStreamMode(m_limelight, StreamType.kPiPSecondary));

                        ShuffleboardLayout ledCommands = Shuffleboard.getTab("Vision")
                                        .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));

                        ledCommands.add("LedsOn", new LimelightLeds(m_limelight, LedMode.kforceOn));
                        ledCommands.add("LedsOff", new LimelightLeds(m_limelight, LedMode.kforceOff));
                        ledCommands.add("LedsBlink", new LimelightLeds(m_limelight, LedMode.kforceBlink));
                        ledCommands.add("LedsPipeline", new LimelightLeds(m_limelight, LedMode.kpipeLine));

                        ShuffleboardLayout visionData = Shuffleboard.getTab("Vision")
                                        .getLayout("Data", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 6)
                                        .withProperties(Map.of("Label position", "LEFT")); //
                        visionData.addBoolean("Connected", () -> m_limelight.isConnected());
                        visionData.addBoolean("TargetVertOK", () -> m_limelight.getHorOnTarget());
                        visionData.addBoolean("TargetHorOK", () -> m_limelight.getVertOnTarget());
                        visionData.addBoolean("TargetFound", () -> m_limelight.getIsTargetFound());
                        visionData.addNumber("DegHToTarget", () -> m_limelight.getdegRotationToTarget());
                        visionData.addNumber("DegVertToTarget", () -> m_limelight.getdegVerticalToTarget());
                        visionData.addNumber("Pipeline #", () -> m_limelight.getPipeline());

                        visionData.addNumber("TargetArea", () -> m_limelight.getTargetArea());
                        visionData.addNumber("BNDBoxWidth", () -> m_limelight.getBoundingBoxWidth());
                        visionData.addNumber("BndBoxHeight", () -> m_limelight.getBoundingBoxHeight());

                        visionData.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);

                        if (RobotBase.isReal()) {

                                LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
                                ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("Vision");
                                driverDisplayTab.add("Limelight", LLFeed).withWidget(BuiltInWidgets.kCameraStream)
                                                .withPosition(5, 0).withSize(5, 4)
                                                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify
                                                                                                                        // widget
                                                                                                                        // properties
                                                                                                                        // here
                        }

                }
                /**
                 * 
                 * Control Panel
                 * 
                 */
                if (m_showControlPanel) {
                        ShuffleboardLayout controlPanelCommands = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("ControlPanel", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(1, 4).withProperties(Map.of("Label position", "Top")); //
                                                                                                         // labels

                        controlPanelCommands.add("ArmRaise", new ControlPanelArm(m_controlPanel, true));
                        controlPanelCommands.add("LookForRevs", new PositionNumberRevs(m_controlPanel, 30, .25));

                        controlPanelCommands.add("ArmLower", new ControlPanelArm(m_controlPanel, false));
                        controlPanelCommands.add("ToggleLookForColor", new ToggleLookForColor(m_controlPanel));

                        controlPanelCommands.add("PositionToColor", new PositionToColor(m_controlPanel, .25));

                        ShuffleboardLayout cpValues = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("CPValues", BuiltInLayouts.kList).withPosition(5, 0).withSize(2, 6)
                                        .withProperties(Map.of("Label position", "Left")); //
                                                                                           // labels

                        cpValues.addNumber("Motor Amps", () -> m_controlPanel.getMotorAmps());
                        cpValues.addNumber("Motor CMD", () -> m_controlPanel.getMotorSet());
                        cpValues.add("CP", m_controlPanel);
                        cpValues.addNumber("SensorDistance", () -> m_controlPanel.getSensorDistance());
                        cpValues.addNumber("IR", () -> m_controlPanel.getSensorIR());
                        cpValues.addNumber("Revs Done", () -> m_controlPanel.revsDone);
                        cpValues.add(m_controlPanel);

                }

                /**
                 * 
                 * Climber
                 * 
                 */
                if (m_showClimber)

                {
                        ShuffleboardLayout climberCommands = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("Climber", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 3)
                                        .withProperties(Map.of("Label position", "TOP"));

                        climberCommands.add("ArmRaise", new InstantCommand(() -> m_climber.raiseArm()));
                        climberCommands.add("ArmLower", new InstantCommand(() -> m_climber.lowerArm()));

                        ShuffleboardLayout climberValues = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("ClimberValues", BuiltInLayouts.kList).withPosition(1, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "TOP")); //
                                                                                                         // labels

                        climberValues.addNumber("Motor Amps", () -> m_climber.getMotorAmps());
                        climberValues.addNumber("Motor Out", () -> m_climber.getMotorOut());
                        climberValues.add("Climber", m_climber);

                }

        }
}
