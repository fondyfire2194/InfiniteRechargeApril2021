// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.CheckCANDevices;
import frc.robot.commands.CellIntake.IntakeArm;
import frc.robot.commands.Climber.ClimberArm;
import frc.robot.commands.ControlPanel.ControlPanelArm;
import frc.robot.commands.ControlPanel.PositionNumberRevs;
import frc.robot.commands.ControlPanel.PositionToColor;
import frc.robot.commands.ControlPanel.ToggleLookForColor;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.RobotDrive.PositionRobotInc;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.ResetPose;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.Shooter.ChangeShooterSpeed;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Shooter.StopShooterWheels;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.ResetTiltAngle;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretInc;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Turret.StopTurret;
import frc.robot.commands.Vision.LimelightCamMode;
import frc.robot.commands.Vision.LimelightLeds;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.LimelightStreamMode;
import frc.robot.commands.Vision.UseVision;
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
        private boolean m_turretTune = false;
        private boolean m_showTilt = true;
        private boolean m_tiltTune = false;
        private boolean m_showShooter = true;
        private boolean m_shooterTune = false;
        private boolean m_showRobot = true;
        private boolean m_robotTune = false;
        private boolean m_showClimberControlPanel = true;

        private boolean m_showTransport = true;
        private boolean m_showVision = true;
        private boolean m_showTrajectory = false;
        private boolean m_showSubsystems = true;
        private boolean m_showPower = true;
        private boolean m_showTiltGains = false;
        private boolean m_showTurretGains = true;
        private boolean m_showButtons = true;
        private HttpCamera LLFeed;
        public double timeToStart;

        public SendableChooser<Integer> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
        SendableChooser<String> driverJS = new SendableChooser<>();
        SendableChooser<String> coDriverXBox = new SendableChooser<>();
        SendableChooser<String> setUpXBox = new SendableChooser<>();
        private ComplexWidget pdpWidget;

        public SetupShuffleboard(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CellTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, ControlPanelSubsystem panel, RearIntakeSubsystem intake,
                        FondyFireTrajectory traj, ClimberSubsystem climber, boolean liveMatch) {
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

                if (true) {
                        // Put
                        // autonomous chooser on the dashboard.
                        // The first argument is the root container
                        // The second argument is whether logging and config should be given separate
                        // tabs
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1) // make the
                                                                                                          // widget
                                        // 2x1
                                        .withPosition(0, 0); // place it in the top-left corner

                        int place = 0;
                        autoChooser.setDefaultOption("Center Start Shoot", place);
                        place = 1;
                        autoChooser.setDefaultOption("Center Start Retract Shoot", place);
                        place = 2;
                        autoChooser.addOption("Left Start Shoot", place);
                        place = 3;
                        autoChooser.addOption("Left Start Retract Shoot", place);
                        place = 4;
                        autoChooser.addOption("Right Start Shoot", place);
                        place = 5;
                        autoChooser.addOption("Right Start Retract Shoot", place);
                        place = 6;
                        autoChooser.addOption("Trench Start Two Pickup Shoot", place);
                        place = 7;
                        autoChooser.addOption("Trench Start One More Pickup Shoot", place);
                        place = 8;
                        autoChooser.addOption("Cross Line", place);

                        Shuffleboard.getTab("Pre-Round").add("Auto Delay", startDelayChooser).withSize(2, 1)
                                        .withPosition(2, 0); //

                        startDelayChooser.setDefaultOption("No Delay", 0);
                        startDelayChooser.addOption("One Second", 1);
                        startDelayChooser.addOption("Two Seconds", 2);
                        startDelayChooser.addOption("Three Seconds", 3);
                        startDelayChooser.addOption("Four Seconds", 4);
                        startDelayChooser.addOption("Five Seconds", 5);


                        ShuffleboardLayout preMatch = Shuffleboard.getTab("Pre-Round")
                                        .getLayout("Info", BuiltInLayouts.kList).withPosition(0, 1).withSize(2, 5)
                                        .withProperties(Map.of("Label position", "TOP"));

                        preMatch.addNumber("TiltView", () -> m_tilt.getAngle());
                        preMatch.add("CANCheck", new CheckCANDevices(this));
                        preMatch.addNumber("TurretView", () -> m_turret.getAngle());

                        preMatch.addBoolean("Tilt Down OK", () -> m_tilt.onLimitSwitch);
                        preMatch.addBoolean("CANConnected",
                                        () -> m_tilt.tiltMotorConnected && m_turret.turretMotorConnected
                                                        && m_transport.allConnected && m_shooter.allConnected
                                                        && m_robotDrive.allConnected && m_climber.climberMotorConnected
                                                        && m_intake.intakeMotorConnected
                                                        && m_controlPanel.controlPanelMotorConnected);

                        if (RobotBase.isReal()) {

                                LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

                                Shuffleboard.getTab("Pre-Round").add("Limelight", LLFeed)
                                                .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 0)
                                                .withSize(3, 2)
                                                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//
                        }

                        /**
                         * 
                         * Competition Driver Tab
                         * 
                         */

                        ShuffleboardLayout competition1 = Shuffleboard.getTab("Competition")
                                        .getLayout("Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));

                        competition1.addNumber("ShootAngle", () -> m_tilt.getAngle());
                        competition1.addNumber("TurretPosn", () -> m_turret.getAngle());
                        competition1.addNumber("RobotPosn", () -> m_robotDrive.getAverageDistance());
                        competition1.addNumber("ShooterSpeed", () -> m_shooter.getRPM());
                        competition1.addNumber("Intake", () -> m_intake.getMotor());
                        competition1.addNumber("FrontRoller", () -> m_transport.getFrontRoller());
                        competition1.addNumber("RearRoller", () -> m_transport.getRearRoller());

                        ShuffleboardLayout shoot = Shuffleboard.getTab("Competition")
                                        .getLayout("Shoot", BuiltInLayouts.kList).withPosition(1, 0).withSize(2, 1)
                                        .withProperties(Map.of("Label position", "HIDDEN"));

                        shoot.addBoolean("SHOOT", () -> (m_shooter.atSpeed() && m_turret.atTargetAngle()
                                        && m_tilt.atTargetAngle()));

                        ShuffleboardLayout competition = Shuffleboard.getTab("Competition")
                                        .getLayout("Values", BuiltInLayouts.kList).withPosition(1, 1).withSize(1, 4)
                                        .withProperties(Map.of("Label position", "TOP"));

                        competition.addBoolean("IntakeArm Up", () -> m_intake.getArmRaised());
                        competition.addBoolean("IntakeArm Down", () -> m_intake.getArmLowered());
                        competition.addBoolean("TiltOnTarget", () -> m_tilt.atTargetAngle());
                        competition.addBoolean("TurretOnTarget", () -> m_turret.atTargetAngle());
                        competition.addBoolean("ShooterAtSpeed", () -> m_shooter.atSpeed());
                        competition.addBoolean("Use Vision", () -> m_limelight.useVision);

                        if (RobotBase.isReal()) {

                                LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
                                ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("Competition");
                                driverDisplayTab.add("Limelight", LLFeed).withWidget(BuiltInWidgets.kCameraStream)
                                                .withPosition(3, 0).withSize(6, 5)
                                                .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify
                                                                                                                        // widget
                                                                                                                        // properties
                        } // here

                }

                /**
                 * 
                 * Shooter Turret
                 * 
                 */
                if (m_showTurret & !liveMatch)

                {
                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("BtnA leftStickY", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                                          // commands

                        turretCommands.add("Reset to 0", new ResetTurretAngle(m_turret));
                        turretCommands.add("Position To 0", new PositionTurret(m_turret, 0));// degrees
                        turretCommands.add("Position To -90", new PositionTurret(m_turret, -90));// degrees
                        turretCommands.add("Position To 90", new PositionTurret(m_turret, 90));
                        turretCommands.add("Position + 2", new PositionTurretInc(m_turret, 2));
                        turretCommands.add("StopTurret", new StopTurret(m_turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(m_turret));
                        turretCommands.add("Cmd", m_turret);
                        turretCommands.addNumber("Faults", () -> m_turret.getFaults());

                        ShuffleboardLayout turretValues = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("TurretValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        turretValues.addNumber("TUAngle", () -> m_turret.getAngle());
                        turretValues.addNumber("TUTgt", () -> m_turret.targetAngle);
                        turretValues.addNumber("Pct", () -> m_turret.getOut());
                        turretValues.addNumber("Amps", () -> m_turret.getAmps());
                        turretValues.addNumber("Speed", () -> m_turret.getSpeed());
                        turretValues.addNumber("Vision Offset", () -> m_turret.targetHorizontalOffset);
                        turretValues.addNumber("AdjTarget", () -> m_turret.adjustedTargetAngle);
                        turretValues.addNumber("Vision Error", () -> m_limelight.getdegRotationToTarget());
                        turretValues.addNumber("IAccum", () -> m_turret.getIaccum());
                        turretValues.addNumber("HorOff+Right", () -> m_limelight.horizontalOffset);

                        ShuffleboardLayout turretValues2 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(2, 3).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels
                        turretValues2.addBoolean("Connected (8)", () -> m_turret.turretMotorConnected);

                        turretValues2.addBoolean("PlusLimit", () -> m_turret.onPlusSoftwareLimit());

                        turretValues2.addBoolean("MinusLimit", () -> m_turret.onMinusSoftwareLimit());

                        turretValues2.addBoolean("SWLimitEn", () -> m_turret.getSoftwareLimitsEnabled());

                        turretValues2.addBoolean("InPosition", () -> m_turret.atTargetAngle());

                        turretValues2.addBoolean("BrakeMode", () -> m_turret.isBrake());
                        turretValues2.addBoolean("TargetHorOK", () -> m_limelight.getHorOnTarget());

                        turretValues2.addBoolean("OKTune", () -> (m_turret.tuneOn && m_turret.lastTuneOn));

                        if (m_turretTune) {

                                ShuffleboardLayout turretValues1 = Shuffleboard.getTab("SetupTurret")
                                                .getLayout("Charts", BuiltInLayouts.kList).withPosition(4, 0)
                                                .withSize(4, 5).withProperties(Map.of("Label position", "TOP")); // labels

                                turretValues1.addNumber("Speed", () -> m_turret.getSpeed())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                turretValues1.addNumber("Position", () -> m_turret.getAngle())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2); //
                                turretValues1.addNumber("Amps", () -> m_turret.getAmps())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2); //
                        }
                }
                /**
                 * Shooter Tilt
                 */
                if (m_showTilt & !liveMatch) {
                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Reset To Min", new ResetTiltAngle(m_tilt));
                        tiltCommands.add("Position To 30",
                                        new PositionTilt(m_tilt, HoodedShooterConstants.TILT_MAX_ANGLE));
                        tiltCommands.add("Position To 15",
                                        new PositionTilt(m_tilt, HoodedShooterConstants.TILT_MID_ANGLE));
                        tiltCommands.add("Position To 0",
                                        new PositionTilt(m_tilt, HoodedShooterConstants.TILT_MIN_ANGLE));

                        tiltCommands.add("PositionToSwitch", new TiltMoveToReverseLimit(m_tilt));
                        tiltCommands.add("StopTilt", new StopTilt(m_tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(m_tilt));
                        tiltCommands.add("Cmd", m_tilt);
                        tiltCommands.addNumber("Faults", () -> m_tilt.faultSeen);
                        tiltCommands.addString("To Jog", () -> "SetupXBox Btn Y left Y");

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TIAngle", () -> m_tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> m_tilt.targetAngle);
                        tiltValues.addNumber("PCT", () -> m_tilt.getOut());
                        tiltValues.addNumber("Amps", () -> m_tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> m_tilt.getSpeed());
                        tiltValues.addNumber("Vision Offset", () -> m_tilt.targetVerticalOffset);
                        tiltValues.addNumber("AdjTarget", () -> m_tilt.adjustedTargetAngle);
                        tiltValues.addNumber("Vision Error", () -> m_limelight.getdegVerticalToTarget());
                        tiltValues.addNumber("MotorDeg", () -> m_tilt.getMotorDegrees());
                        tiltValues.addNumber("MotorTarget", () -> m_tilt.motorEndpointDegrees);
                        tiltValues.addNumber("VertOff+Low", () -> m_limelight.verticalOffset);
                        // tiltValues.addNumber("CameraAngle", () -> m_tilt.getCameraAngle());

                        tiltValues.addNumber("IAccum", () -> m_tilt.getIaccum());

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 3).withSize(4, 2)
                                        .withProperties(Map.of("Label position", "TOP")); // labels

                        tiltValues2.addBoolean("InPosition", () -> m_tilt.atTargetAngle());

                        tiltValues2.addBoolean("OnBottomLS", () -> m_tilt.m_reverseLimit.get());

                        tiltValues2.addBoolean("PosResetDone", () -> m_tilt.positionResetDone);
                        tiltValues2.addBoolean("BrakeMode", () -> m_tilt.isBrake());
                        tiltValues2.addBoolean("OKTune", () -> (m_tilt.tuneOn && m_tilt.lastTuneOn));
                        tiltValues2.addBoolean("Connected (9)", () -> m_tilt.tiltMotorConnected);
                        tiltValues2.addBoolean("+SWLimit", () -> m_tilt.onPlusSoftwareLimit());
                        tiltValues2.addBoolean("-SWLimit", () -> m_tilt.onMinusSoftwareLimit());
                        tiltValues2.addBoolean("SWLimitEn", () -> m_tilt.getSoftwareLimitsEnabled());
                        tiltValues2.addBoolean("TargetVertOK", () -> m_limelight.getVertOnTarget());

                        if (m_tiltTune) {

                                ShuffleboardLayout tiltValues1 = Shuffleboard.getTab("SetupTilt")
                                                .getLayout("Charts", BuiltInLayouts.kList).withPosition(4, 0)
                                                .withSize(4, 3).withProperties(Map.of("Label position", "TOP")); // labels
                                                                                                                 // for
                                tiltValues1.addNumber("Speed", () -> m_tilt.getSpeed())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                tiltValues1.addNumber("Position", () -> m_tilt.getAngle())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                tiltValues1.addNumber("Amps", () -> m_tilt.getAmps()).withWidget(BuiltInWidgets.kGraph)
                                                .withSize(4, 2);
                        }
                }
                /**
                 * 
                 * Shooter and Transport
                 * 
                 */
                if (m_showShooter & !liveMatch) {
                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("MAXFPS 120", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                           // commands

                        shooterCommands.add("Shooter 50 mps", new StartShooterWheels(m_shooter, 50));
                        shooterCommands.add("Stop", new StopShooterWheels(m_shooter));
                        shooterCommands.add("Stop Shoot", new StopShoot(m_shooter, m_transport));
                        shooterCommands.add("Inc 2 mpersec", new ChangeShooterSpeed(m_shooter, 2));
                        shooterCommands.add("Dec 2 mpersec ", new ChangeShooterSpeed(m_shooter, -2));
                        shooterCommands.add("Shoot", new ShootCells(m_shooter, m_transport, m_compressor, 0));
                        shooterCommands.add("ClearFaults", new ClearShFaults(m_shooter));
                        shooterCommands.add("Cmd", m_shooter);

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        shooterValues.addNumber("LeftMPS", () -> m_shooter.getMPS());
                        shooterValues.addNumber("Left PCT", () -> m_shooter.getLeftPctOut());
                        shooterValues.addNumber("LeftAmps", () -> m_shooter.getLeftAmps());
                        shooterValues.addNumber("RightAmps", () -> m_shooter.getRightAmps());
                        shooterValues.addNumber("SpeedCommand FPS", () -> m_shooter.requiredMps);
                        shooterValues.addNumber("LeftFaults", () -> m_shooter.getLeftFaults());
                        shooterValues.addNumber("RightFaults", () -> m_shooter.getRightFaults());
                        shooterValues.addBoolean("CameraHasSpeed", () -> m_shooter.useCameraSpeed);

                        ShuffleboardLayout shooterValues1 = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues1", BuiltInLayouts.kList).withPosition(2, 3)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues1.addNumber("VertOffset", () -> m_tilt.targetVerticalOffset);
                        shooterValues1.addNumber("HorOffset", () -> m_turret.targetHorizontalOffset);

                        shooterValues1.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        shooterValues1.addNumber("CameraAngle", () -> m_tilt.getCameraAngle());

                        ShuffleboardLayout shooterValues2 = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels

                        shooterValues2.addBoolean("AtSpeed", () -> m_shooter.atSpeed());
                        shooterValues2.addBoolean("CameraBypassed", () -> m_shooter.cameraSpeedBypassed);
                        shooterValues2.addBoolean("TuneOn", () -> (m_shooter.tuneOn && m_shooter.lastTuneOn));
                        shooterValues2.addBoolean("LeftConnected (6)", () -> m_shooter.leftMotorConnected);
                        shooterValues2.addBoolean("RightConnected (7)", () -> m_shooter.rightMotorConnected);
                        shooterValues2.addBoolean(("RollersTurning"),
                                        () -> Math.abs(m_transport.getFrontRoller()) > .1);

                        if (m_shooterTune) {

                                ShuffleboardLayout shooterValues3 = Shuffleboard.getTab("SetupShooter")
                                                .getLayout("Charts", BuiltInLayouts.kList).withPosition(4, 0)
                                                .withSize(4, 5).withProperties(Map.of("Label position", "LEFT")); // labels
                                shooterValues3.addNumber("Speed", () -> m_shooter.getRPM())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                shooterValues3.addNumber("Amps", () -> m_shooter.getLeftAmps())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);

                        }
                }

                if (m_showTransport & !liveMatch) {

                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupCellHandle")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels
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

                        ShuffleboardLayout transportValues1 = Shuffleboard.getTab("SetupCellHandle")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(2, 0).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "TOP")); // label

                        transportValues1.addBoolean("Arm Up", () -> m_intake.getArmRaised());
                        transportValues1.addBoolean("Arm Down", () -> m_intake.getArmLowered());
                        transportValues1.addBoolean("IntakeConnected (10)", () -> m_intake.intakeMotorConnected);

                        transportValues1.addBoolean("LeftBeltConnected (13)", () -> m_transport.leftBeltMotorConnected);
                        transportValues1.addBoolean("RightBeltConnected (11)",
                                        () -> m_transport.rightBeltMotorConnected);
                        transportValues1.addBoolean("RearRollerConnected (12)",
                                        () -> m_transport.rearRollerMotorConnected);
                        transportValues1.addBoolean("FrontRollerConnected (14)",
                                        () -> m_transport.frontRollerMotorConnected);

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("SetupCellHandle")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeValues.addBoolean("Arm Up", () -> m_intake.getArmRaised());
                        intakeValues.addBoolean("Arm Down", () -> m_intake.getArmLowered());
                        intakeValues.addNumber("Motor Amps", () -> m_intake.getMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> m_intake.getMotor());
                        intakeValues.add("ArmRaise", new IntakeArm(m_intake, false));
                        intakeValues.add("ArmLower", new IntakeArm(m_intake, true));

                }
                /**
                 * 
                 * Robot
                 * 
                 */
                if (m_showRobot & !liveMatch) {
                        ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        robotCommands.add("Reset Enc", new ResetEncoders(m_robotDrive));
                        robotCommands.add("Reset Gyro", new ResetGyro(m_robotDrive));
                        robotCommands.add("Reset Pose", new ResetPose(m_robotDrive));
                        robotCommands.add("Pos to 10M", new PositionRobot(m_robotDrive, 10.));
                        robotCommands.add("Pos to 0M", new PositionRobot(m_robotDrive, 0));
                        robotCommands.add("Pos +1", new PositionRobotInc(m_robotDrive, 1));
                        robotCommands.add("Pos -1", new PositionRobotInc(m_robotDrive, -1));
                        // robotCommands.add("Rot to 90", new TurnToAngleProfiled(m_robotDrive, 90));
                        // robotCommands.add("Rot to 0", new TurnToAngleProfiled(m_robotDrive, 0));
                        // robotCommands.add("Rot to -90", new TurnToAngleProfiled(m_robotDrive, -90));
                        robotCommands.add("ClearFaults", new ClearRobFaults(m_robotDrive));
                        robotCommands.add("Stop Robot", new StopRobot(m_robotDrive));
                        robotCommands.add("Cmd", m_robotDrive);

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
                        robotValues.addNumber("Target", () -> m_robotDrive.leftTargetPosition);

                        ShuffleboardLayout robotValues2 = Shuffleboard.getTab("SetupRobot")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(8, 3).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "TOP")); // labels

                        robotValues2.addBoolean("TuneOn", () -> (m_robotDrive.tuneOn && m_robotDrive.lastTuneOn));
                        robotValues2.addBoolean("Left1Connected (2)", () -> m_robotDrive.leftLeadConnected);
                        robotValues2.addBoolean("Left2Connected (3)", () -> m_robotDrive.leftFollowerConnected);
                        robotValues2.addBoolean("Right1Connected (4)", () -> m_robotDrive.rightLeadConnected);
                        robotValues2.addBoolean("Right2Connected (5)", () -> m_robotDrive.rightFollowerConnected);
                        robotValues2.addBoolean("LInPosition", () -> m_robotDrive.getInPositionLeft());
                        robotValues2.addBoolean("RInPosition", () -> m_robotDrive.getInPositionRight());

                        if (m_robotTune) {

                                ShuffleboardLayout robotValues1 = Shuffleboard.getTab("SetupRobot")
                                                .getLayout("Charts", BuiltInLayouts.kList).withPosition(4, 0)
                                                .withSize(4, 4).withProperties(Map.of("Label position", "LEFT"));

                                robotValues1.addNumber("LeftSpeed", () -> m_robotDrive.getLeftRate())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                robotValues1.addNumber("Position", () -> m_robotDrive.getAverageDistance())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);
                                robotValues1.addNumber("Amps", () -> m_robotDrive.getLeftAmps())
                                                .withWidget(BuiltInWidgets.kGraph).withSize(4, 2);

                        }
                }
                /**
                 * 
                 * Run Trajectory
                 * 
                 */
                if (m_showTrajectory & !liveMatch) {
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

                        ShuffleboardLayout scheduler = Shuffleboard.getTab("Subsystems")
                                        .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(3, 0).withSize(6, 2)
                                        .withProperties(Map.of("Label position", "TOP")); //

                        scheduler.add("Scheduler", CommandScheduler.getInstance());

                        ShuffleboardLayout canBus = Shuffleboard.getTab("Subsystems")
                                        .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(3, 2).withSize(5, 2)
                                        .withProperties(Map.of("Label position", "TOP")); // labels

                        canBus.addBoolean("TurretConnected (8)", () -> m_turret.turretMotorConnected);
                        canBus.addBoolean("TiltConnected (9)", () -> m_tilt.tiltMotorConnected);
                        canBus.addBoolean("LeftShooterConnected (6)", () -> m_shooter.leftMotorConnected);
                        canBus.addBoolean("RightShooterConnected (7)", () -> m_shooter.rightMotorConnected);
                        canBus.addBoolean("LeftBeltConnected (13)", () -> m_transport.leftBeltMotorConnected);
                        canBus.addBoolean("RightBeltConnected (11)", () -> m_transport.rightBeltMotorConnected);
                        canBus.addBoolean("RearRollerConnected (12)", () -> m_transport.rearRollerMotorConnected);
                        canBus.addBoolean("FrontRollerConnected (14)", () -> m_transport.frontRollerMotorConnected);
                        canBus.addBoolean("CLConected (15)", () -> m_climber.climberMotorConnected);
                        canBus.addBoolean("LDR1Connected (2)", () -> m_robotDrive.leftLeadConnected);
                        canBus.addBoolean("LDr2Connected (3)", () -> m_robotDrive.leftFollowerConnected);
                        canBus.addBoolean("RDr1Connected (4)", () -> m_robotDrive.rightLeadConnected);
                        canBus.addBoolean("RDr2Connected (5)", () -> m_robotDrive.rightFollowerConnected);
                        canBus.addBoolean("CPConnected (16)", () -> m_controlPanel.controlPanelMotorConnected);
                        canBus.addBoolean("IntakeConnected (10)", () -> m_intake.intakeMotorConnected);

                }
                /**
                 * 
                 * Vision
                 * 
                 */
                if (m_showVision & !liveMatch) {
                        ShuffleboardLayout zoomCommands = Shuffleboard.getTab("Vision")
                                        .getLayout("Zoom", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "TOP")); //
                                                                                          // labels
                                                                                          // for

                        zoomCommands.add("No Zoom", new LimelightSetPipeline(m_limelight, 0));
                        zoomCommands.add("2XZoom", new LimelightSetPipeline(m_limelight, 1));
                        zoomCommands.add("3X Zoom", new LimelightSetPipeline(m_limelight, 2));

                        ShuffleboardLayout visionCommands = Shuffleboard.getTab("Vision")
                                        .getLayout("On-Off", BuiltInLayouts.kList).withPosition(0, 2).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "TOP")); //

                        visionCommands.add("Vision On", new UseVision(limelight, true));
                        visionCommands.add("Vision Off", new UseVision(limelight, false));

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
                                        .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 3)
                                        .withProperties(Map.of("Label position", "TOP"));

                        ledCommands.add("LedsOn", new LimelightLeds(m_limelight, LedMode.kforceOn));
                        ledCommands.add("LedsOff", new LimelightLeds(m_limelight, LedMode.kforceOff));
                        ledCommands.add("LedsBlink", new LimelightLeds(m_limelight, LedMode.kforceBlink));
                        ledCommands.add("LedsPipeline", new LimelightLeds(m_limelight, LedMode.kpipeLine));

                        ShuffleboardLayout visionData = Shuffleboard.getTab("Vision")
                                        .getLayout("Data", BuiltInLayouts.kList).withPosition(3, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        visionData.addNumber("DegHToTarget", () -> m_limelight.getdegRotationToTarget());
                        visionData.addNumber("DegVertToTarget", () -> m_limelight.getdegVerticalToTarget());
                        visionData.addNumber("Pipeline #", () -> m_limelight.getPipeline());

                        visionData.addNumber("TargetArea", () -> m_limelight.getTargetArea());
                        visionData.addNumber("BNDBoxWidth", () -> m_limelight.getBoundingBoxWidth());
                        visionData.addNumber("BndBoxHeight", () -> m_limelight.getBoundingBoxHeight());
                        visionData.addNumber("CameraAngle", () -> m_tilt.getAngle());
                        visionData.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        visionData.addNumber("CameraCalculatedMPS", () -> m_shooter.cameraCalculatedSpeed);
                        visionData.addNumber("VertOff+Low", () -> m_limelight.verticalOffset);
                        visionData.addNumber("HorOff+Right", () -> m_limelight.horizontalOffset);

                        ShuffleboardLayout visionBools = Shuffleboard.getTab("Vision")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(3, 3).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "TOP")); // labels

                        visionBools.addBoolean("Connected", () -> m_limelight.isConnected());

                        visionBools.addBoolean("TargetVertOK", () -> m_limelight.getVertOnTarget());

                        visionBools.addBoolean("TargetHorOK", () -> m_limelight.getHorOnTarget());

                        visionBools.addBoolean("TargetFound", () -> m_limelight.getIsTargetFound());

                        visionBools.addBoolean("Use Vision", () -> m_limelight.useVision);

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
                if (m_showClimberControlPanel) {

                        ShuffleboardLayout controlPanelCommands = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("ControlPanel", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "Top")); //
                                                                                                         // labels

                        controlPanelCommands.add("ArmRaise", new ControlPanelArm(m_controlPanel, false));
                        controlPanelCommands.add("LookForRevs", new PositionNumberRevs(m_controlPanel, 30, .25));

                        controlPanelCommands.add("ArmLower", new ControlPanelArm(m_controlPanel, true));
                        controlPanelCommands.add("ToggleLookForColor", new ToggleLookForColor(m_controlPanel));

                        controlPanelCommands.add("PositionToColor", new PositionToColor(m_controlPanel, .25));

                        controlPanelCommands.add("CP", m_controlPanel);

                        ShuffleboardLayout cpValues = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("CPValues", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "Left")); //
                                                                                           // labels

                        cpValues.addNumber("Motor Amps", () -> m_controlPanel.getMotorAmps());
                        cpValues.addNumber("Motor CMD", () -> m_controlPanel.getMotorSet());

                        cpValues.addNumber("SensorDistance", () -> m_controlPanel.getSensorDistance());
                        cpValues.addNumber("IR", () -> m_controlPanel.getSensorIR());
                        cpValues.addNumber("Revs Done", () -> m_controlPanel.revsDone);
                        cpValues.addString("GameColor", () -> m_controlPanel.gameData);
                        cpValues.addString("SensorColor", () -> m_controlPanel.colorSeen);

                        cpValues.addBoolean("CP Arm Up", () -> m_controlPanel.getArmRaised());
                        cpValues.addBoolean("CP Arm Down", () -> m_controlPanel.getArmLowered());
                        cpValues.addBoolean("Connected (16)", () -> m_controlPanel.controlPanelMotorConnected);

                        /**
                         * 
                         * Climber
                         * 
                         */

                        ShuffleboardLayout climberCommands = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("Climber", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP"));

                        climberCommands.add("ArmRaise", new ClimberArm(m_climber, false));
                        climberCommands.add("ArmLower", new ClimberArm(m_climber, true));
                        climberCommands.add("Climber", m_climber);

                        ShuffleboardLayout climberValues = Shuffleboard.getTab("SetupClimber_CP")
                                        .getLayout("ClimberValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); //
                                                                                                         // labels
                        climberValues.addBoolean("Climber Arm Down", () -> m_climber.getArmLowered());
                        climberValues.addNumber("Motor Amps", () -> m_climber.getMotorAmps());
                        climberValues.addNumber("Motor Out", () -> m_climber.getMotorOut());
                        climberValues.addBoolean("Connected (15)", () -> m_climber.climberMotorConnected);
                        climberValues.addBoolean("Climber Arm Up", () -> m_climber.getArmRaised());

                }
                if (m_showPower && !liveMatch) {
                        ShuffleboardTab misc = Shuffleboard.getTab("Power");
                        PowerDistributionPanel pdp = new PowerDistributionPanel();
                        pdpWidget = misc.add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistributionPanel)
                                        .withPosition(0, 0).withSize(6, 4);

                }

                if (m_showButtons && !liveMatch) {

                        Shuffleboard.getTab("ButtonsHelp").add("Driver JS", driverJS).withSize(2, 1)

                                        .withPosition(0, 0); // place it in the top-left corner

                        driverJS.setDefaultOption("1 - Hold Pick Up Cells", "1");
                        driverJS.addOption("2 - Hold Shoot Cells", "2");
                        driverJS.addOption("3 - Stop Shooter Wheels", "3");
                        driverJS.addOption("4 - Toggle Camera", "4");
                        driverJS.addOption("5 - Run Shooter Wheels", "5");
                        driverJS.addOption("6 - Not Assigned", "6");

                        driverJS.addOption("7 - Trench Shoot", "7");
                        driverJS.addOption("8 - Straight Shoot", "8");
                        driverJS.addOption("9 - Left Shoot", "9");
                        driverJS.addOption("10 - Right Shoot", "10");
                        driverJS.addOption("11 - Not Assigned", "11");
                        driverJS.addOption("12 - Not Assigned", "12");

                        driverJS.addOption("POV Up - Inc Shooter Speed", "13");
                        driverJS.addOption("POV Down - Dec Shooter Speed", "14'");
                        driverJS.addOption("POV Left - Not Assigned", "15");
                        driverJS.addOption("POV Right - Not Assigned", "16");

                        Shuffleboard.getTab("ButtonsHelp").add("CoDriverXBox", coDriverXBox).withSize(2, 1)
                                        .withPosition(3, 0); //
                        coDriverXBox.setDefaultOption("A - Lower Color Arm", "1");
                        coDriverXBox.addOption("B - Raise Color Arm", "2");

                        coDriverXBox.addOption("X - Turn Color Wheel", "3");
                        coDriverXBox.addOption("Y - CP Position To Color", "4");

                        coDriverXBox.addOption("Back - CP Position Number Turns", "5");
                        coDriverXBox.addOption("Start - Not Assigned", "6");

                        coDriverXBox.addOption("Left Trigger - Not Assigned", "7");
                        coDriverXBox.addOption("Right Trigger - Not Assigned", "8");

                        coDriverXBox.addOption("Right Bumper - Not Assigned", "9");
                        coDriverXBox.addOption("Left Bumper - Not Assigned", "10");

                        coDriverXBox.addOption("POV Up - Not Assigned", "11");
                        coDriverXBox.addOption("POV Down - Not Assigned", "12");
                        coDriverXBox.addOption("POV Left - Not Assigned", "13");
                        coDriverXBox.addOption("POV Right - Not Assigned", "14");

                        Shuffleboard.getTab("ButtonsHelp").add("SetupXBox", setUpXBox).withSize(2, 1)

                                        .withPosition(6, 0); //

                        setUpXBox.setDefaultOption("A - Hold Jog Turret LeftX", "1");

                        setUpXBox.addOption("B - Not Assigned", "2");
                        setUpXBox.addOption("X - Hold Jog Shooter RightY", "3");

                        setUpXBox.addOption("Y - Hold Jog Tilt LeftY", "4");

                        setUpXBox.addOption("Back - Intake Cells", "5");
                        setUpXBox.addOption("Start - TiltToSwitch", "6");

                        setUpXBox.addOption("Left Trigger - Climb", "7");
                        setUpXBox.addOption("Right Trigger - Return Climber", "8");

                        setUpXBox.addOption("Right Bumper - Hold Bypass Soft Limits", "9");
                        setUpXBox.addOption("Left Bumper - Not Assigned", "10");

                        setUpXBox.addOption("POV Up - Hold Run Belts", "11");
                        setUpXBox.addOption("POV Down - Hold Run Rollers", "12");
                        setUpXBox.addOption("POV Left - Hold Pulse Left Belt", "13");
                        setUpXBox.addOption("POV Right - Hold Pulse Right Belt", "14");

                }

        }

        public void checkCANDevices() {
                m_turret.checkCAN();
                m_tilt.checkCAN();
                m_intake.checkCAN();
                m_shooter.checkCAN();
                m_robotDrive.checkCAN();
                m_transport.checkCAN();
                m_controlPanel.checkCAN();
                m_climber.checkCAN();
        }
}
