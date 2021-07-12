// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.AutoCommands.PowerPort.CenterPowerPortToTargetOnly;
import frc.robot.commands.AutoCommands.ShieldGenOne.ToShieldGenTarget;
import frc.robot.commands.AutoCommands.TrenchOne.ToTrenchTarget;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.CellIntake.StopIntakeMotor;
import frc.robot.commands.CellTransport.HoldCell;
import frc.robot.commands.CellTransport.LowerLeftArm;
import frc.robot.commands.CellTransport.RaiseLeftArm;
import frc.robot.commands.CellTransport.ReleaseCell;
import frc.robot.commands.CellTransport.ReleaseOneCell;
import frc.robot.commands.CellTransport.RunRollers;
import frc.robot.commands.CellTransport.StopRollers;
import frc.robot.commands.RobotDrive.ClearRobFaults;
import frc.robot.commands.RobotDrive.EndDriveLog;
import frc.robot.commands.RobotDrive.LogDriveData;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.PickupMoveVelocity;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.StopRobot;
import frc.robot.commands.Shooter.ChooseShooterSpeedSource;
import frc.robot.commands.Shooter.ClearShFaults;
import frc.robot.commands.Shooter.EndShootLog;
import frc.robot.commands.Shooter.LogDistanceData;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.ClearFaults;
import frc.robot.commands.Tilt.EndTiltLog;
import frc.robot.commands.Tilt.LogTiltData;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.StopTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.ClearTurFaults;
import frc.robot.commands.Turret.EndTurretLog;
import frc.robot.commands.Turret.LogTurretData;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.ResetTurretAngle;
import frc.robot.commands.Turret.StopTurret;
import frc.robot.commands.Vision.LimelightCamMode;
import frc.robot.commands.Vision.LimelightLeds;
import frc.robot.commands.Vision.LimelightSetPipeline;
import frc.robot.commands.Vision.LimelightStreamMode;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.CellTransportSubsystem;
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
        private final RearIntakeSubsystem m_intake;
        private boolean m_showTurret = true;
        private boolean m_showTilt = true;
        private boolean m_showShooter = true;
        private boolean m_showRobot = true;
        private boolean m_showTransport = true;
        private boolean m_showVision = true;
        private boolean m_showSubsystems = false;
        private HttpCamera LLFeed;
        private UsbCamera intakeFeed;
        public double timeToStart;

        public SendableChooser<Integer> autoChooser = new SendableChooser<>();
        public SendableChooser<Integer> startDelayChooser = new SendableChooser<>();
        SendableChooser<String> driverJS = new SendableChooser<>();
        SendableChooser<String> coDriverXBox = new SendableChooser<>();
        SendableChooser<String> setUpXBox = new SendableChooser<>();
        private ComplexWidget pdpWidget;

        public SetupShuffleboard(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
                        RevShooterSubsystem shooter, CellTransportSubsystem transport, Compressor compressor,
                        LimeLight limelight, RearIntakeSubsystem intake, FondyFireTrajectory traj, boolean liveMatch) {
                m_turret = turret;
                m_tilt = tilt;
                m_robotDrive = drive;
                m_transport = transport;
                m_compressor = compressor;
                m_shooter = shooter;
                m_limelight = limelight;
                m_intake = intake;

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
                        Shuffleboard.getTab("Pre-Round").add("Auto Commands", autoChooser).withSize(2, 1)
                                        .withPosition(0, 0); // place it in the top-left corner

                        autoChooser.setDefaultOption("Cross Line", 0);

                        autoChooser.addOption("Center Start Retract Shoot", 1);

                        autoChooser.addOption("Trench 3 M 1", 2);

                        autoChooser.addOption("Trench 3 M 2", 3);

                        autoChooser.addOption("Trench 3 M 3", 4);

                        autoChooser.addOption("ShieldGen 3 M 1", 5);

                        autoChooser.addOption("Shield Gen 3 M 2", 6);

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
                        preMatch.addNumber("TurretView", () -> m_turret.getAngle());

                        preMatch.addBoolean("Tilt Down OK", () -> m_tilt.m_reverseLimit.get());
                        preMatch.addBoolean("CellArmUp", () -> m_transport.getCellArmUp());
                        preMatch.addBoolean("CellAtShoot", () -> m_transport.getBallAtShoot());

                        preMatch.addBoolean("CANConnected",
                                        () -> m_tilt.tiltMotorConnected && m_turret.turretMotorConnected
                                                        && m_transport.allConnected && m_shooter.allConnected
                                                        && m_robotDrive.allConnected && m_intake.intakeMotorConnected);

                        ShuffleboardLayout logCmd = Shuffleboard.getTab("Pre-Round")
                                        .getLayout("LogCommands", BuiltInLayouts.kList).withPosition(2, 1)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                        logCmd.add("LogTilt", new LogTiltData(tilt, limelight));
                        logCmd.add("LogTurret", new LogTurretData(turret, limelight));
                        // logCmd.add("LogShoot", new LogShootData(shooter, transport));
                        logCmd.add("LogDrive", new LogDriveData(m_robotDrive));
                        logCmd.add("EndLogTilt", new EndTiltLog(tilt));
                        logCmd.add("EndLogTurret", new EndTurretLog(turret));
                        logCmd.add("EndLogShoot", new EndShootLog(shooter));
                        logCmd.add("EndDriveLog", new EndDriveLog(drive));

                        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
                        Shuffleboard.getTab("Pre-Round").add("Limelight", LLFeed)
                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 0).withSize(3, 2)
                                        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//

                        intakeFeed = new UsbCamera("Intake", "http://roboRIO-2194-FRC.local:1180/stream.mjpg");
                        intakeFeed.setResolution(320, 240);

                        ShuffleboardLayout intakeValues = Shuffleboard.getTab("Intake")
                                        .getLayout("IntakeValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "TOP"));

                        intakeValues.addBoolean("Arm Up", () -> m_intake.getArmRaised());
                        intakeValues.addBoolean("Arm Down", () -> m_intake.getArmLowered());
                        intakeValues.addNumber("Motor Amps", () -> m_intake.getMotorAmps());
                        intakeValues.addNumber("Motor CMD", () -> m_intake.getMotor());
                        intakeValues.add("ArmRaise", new IntakeArmRaise(m_intake));
                        intakeValues.add("ArmLower", new IntakeArmLower(m_intake));
                        intakeValues.add("Run Motor", new RunIntakeMotor(intake, .75));
                        intakeValues.add("Stop Motor", new StopIntakeMotor(intake));

                        Shuffleboard.getTab("Intake").add("Intake", intakeFeed).withWidget(BuiltInWidgets.kCameraStream)
                                        .withPosition(2, 0).withSize(6, 4)
                                        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//

                        Shuffleboard.getTab("Pre-Round").add("Intake", intakeFeed)
                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 2).withSize(3, 2)
                                        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));//

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
                        competition1.addNumber("ShooterSetSpeed", () -> m_shooter.requiredMps);
                        competition1.addNumber("ShooterSpeed", () -> m_shooter.getMPS());
                        competition1.addNumber("TargetVertOffsetDeg", () -> m_tilt.targetVerticalOffset);
                        competition1.addNumber("TargetHorOffsetDeg", () -> m_turret.targetHorizontalOffset);
                        competition1.addNumber("CellsShot", () -> m_transport.cellsShot);

                        ShuffleboardLayout competition = Shuffleboard.getTab("Competition")
                                        .getLayout("ShootConditions", BuiltInLayouts.kGrid).withPosition(1, 0)
                                        .withSize(2, 2).withProperties(Map.of("Label position", "TOP"));

                        competition.addBoolean("TiltOnTarget",
                                        () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));
                        competition.addBoolean("TurretOnTarget",
                                        () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));
                        competition.addBoolean("ShooterAtSpeed", () -> m_shooter.atSpeed());
                        competition.addBoolean("Use Vision", () -> m_limelight.useVision);
                        competition.addBoolean("RollersAtSpeed", () -> m_transport.rollersAtSpeed);

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
                {
                        ShuffleboardLayout miscComp = Shuffleboard.getTab("CompetitionMisc")
                                        .getLayout("Misc1", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        miscComp.add("Reset to 0", new ResetTurretAngle(m_turret));
                        miscComp.addNumber("TUAngle", () -> m_turret.getAngle());
                        miscComp.addNumber("TiltAngle", () -> m_tilt.getAngle());
                        miscComp.addNumber("LeftMPS", () -> m_shooter.getMPS());
                        miscComp.addNumber("LeftAmps", () -> m_shooter.getLeftAmps());
                        miscComp.addNumber("RightRPM", () -> m_shooter.getRightRPM());
                        miscComp.addNumber("RightAmps", () -> m_shooter.getRightAmps());
                        miscComp.addNumber("TargetArea%Scrn", () -> m_limelight.getTargetArea());
                        miscComp.addNumber("BNDBoxWidth", () -> m_limelight.getBoundingBoxWidth());
                        miscComp.addNumber("BndBoxHeight", () -> m_limelight.getBoundingBoxHeight());
                        miscComp.addNumber("AspectRatio", () -> m_limelight.getAspectRatio());

                        miscComp.addNumber("RQDMPS", () -> m_shooter.requiredMps);

                        ShuffleboardLayout misComp1 = Shuffleboard.getTab("CompetitionMisc")
                                        .getLayout("Misc2", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        misComp1.add("Hold Cell", new HoldCell(transport));
                        misComp1.add("ReleaseOneCell", new ReleaseOneCell(transport));
                        misComp1.addNumber(("CellArmAngle"), () -> m_transport.getArmAngle());
                        misComp1.addNumber(("CellArmPosn"), () -> m_transport.getArmPosition());
                        // for

                        misComp1.add("Release Cell", new ReleaseCell(transport));
                        misComp1.add("RaiseLeft", new RaiseLeftArm(transport));
                        misComp1.add("LowerLeft", new LowerLeftArm(transport));
                        misComp1.addBoolean("BallAtShoot", () -> m_transport.getBallAtShoot());
                        misComp1.addBoolean("BallAtLeft", () -> m_transport.getBallAtLeft());
                        misComp1.addBoolean("Arm Up", () -> m_transport.getCellArmUp());
                        misComp1.addBoolean("Arm Down", () -> m_transport.getCellArmDown());
                        misComp1.addBoolean("Left Arm Up", () -> m_transport.getLeftArmUp());
                        misComp1.addBoolean("Left Arm Down", () -> m_transport.getLeftArmDown());

                        ShuffleboardLayout misComp2 = Shuffleboard.getTab("CompetitionMisc")
                                        .getLayout("Misc3", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        misComp2.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        misComp2.addNumber("CameraCalcSpeed", () -> m_shooter.cameraCalculatedSpeed);
                        misComp2.addNumber("CameraCalcTilt", () -> m_tilt.cameraCalculatedTiltOffset);
                        misComp2.add("Reset Enc", new ResetEncoders(m_robotDrive));
                        misComp2.add("Reset Gyro", new ResetGyro(m_robotDrive));
                        misComp2.addNumber("LeftMeters", () -> m_robotDrive.getLeftDistance());
                        misComp2.addNumber("RightMeters", () -> m_robotDrive.getRightDistance());

                        misComp2.add("To P-P Target", new CenterPowerPortToTargetOnly(m_turret, m_tilt, m_limelight));

                        misComp2.add("To Trench Target", new ToTrenchTarget(m_turret, m_tilt, m_limelight));

                        misComp2.add("To ShieldGen Target", new ToShieldGenTarget(m_turret, m_tilt, m_limelight));

                        ShuffleboardLayout misComp3 = Shuffleboard.getTab("CompetitionMisc")
                                        .getLayout("Misc4", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));

                        misComp3.addBoolean("RobotStopped", () -> m_robotDrive.robotStoppedForOneSecond);
                        misComp3.addBoolean("BallAtShoot", () -> m_transport.getBallAtShoot());
                        misComp3.addBoolean("BallAtLeft", () -> m_transport.getBallAtLeft());
                        misComp3.addBoolean("No BallAtShoot", () -> m_transport.noBallatShooterForOneSecond);
                        misComp3.addBoolean("NoBallAtLeft", () -> m_transport.noBallatLeftForOneSecond);
                        misComp3.addBoolean("CellAvailable", () -> m_transport.cellAvailable);
                        misComp3.addBoolean("IsShooting", () -> m_shooter.isShooting);

                        ShuffleboardLayout misComVis = Shuffleboard.getTab("CompetitionMisc")
                                        .getLayout("MiscVis", BuiltInLayouts.kList).withPosition(8, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT"));
                        misComVis.add("No Zoom Power Port Pipeline", new LimelightSetPipeline(m_limelight, 1));
                        misComVis.add("No Zoom Shield Gen Pipeline", new LimelightSetPipeline(m_limelight, 2));
                        misComVis.add("No Zoom Trench Pipeline", new LimelightSetPipeline(m_limelight, 3));
                        misComVis.add("Driver Pipeline", new LimelightSetPipeline(m_limelight, 0));
                        misComVis.add("Vision On", new UseVision(limelight, true));
                        misComVis.add("Vision Off", new UseVision(limelight, false));
                        misComVis.addBoolean("LLTGT", () -> m_limelight.getIsTargetFound());
                        misComVis.addBoolean("TiVT", () -> m_tilt.validTargetSeen);
                        misComVis.addBoolean("TuVT", () -> m_turret.validTargetSeen);

                        // Shuffleboard.getTab("Competition").addNumber("TimeRemaining", () ->
                        // m_robotDrive.getMatchTime())
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 0).withSize(1, 1);
                        // Shuffleboard.getTab("Competition").addNumber("Battery", () ->
                        // getPDPInfo()[0])
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 1).withSize(1, 1);
                        // Shuffleboard.getTab("Competition").addNumber("TotalEnegy Ah", () ->
                        // getPDPInfo()[2])
                        // .withWidget(BuiltInWidgets.kTextView).withPosition(9, 2).withSize(1, 1);

                }

                /**
                 * 
                 * Shooter Turret
                 * 
                 */
                if (m_showTurret && !liveMatch)

                {
                        ShuffleboardLayout turretCommands = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                           // commands

                        turretCommands.add("Reset to 0", new ResetTurretAngle(m_turret));
                        turretCommands.add("Position To 0", new PositionTurret(m_turret, 0));// degrees
                        turretCommands.add("Position To -30", new PositionTurret(m_turret, -30));// degrees
                        turretCommands.add("Position To 30", new PositionTurret(m_turret, 30));
                        turretCommands.add("PositionToVision +10",
                                        new PositionTurretToVision(m_turret, m_limelight, 10));
                        turretCommands.add("PositionToVision -10",
                                        new PositionTurretToVision(m_turret, m_limelight, -10));

                        turretCommands.add("StopTurret", new StopTurret(m_turret));
                        turretCommands.add("ClearFaults", new ClearTurFaults(m_turret));
                        turretCommands.add("Cmd", m_turret);
                        turretCommands.addNumber("Faults", () -> m_turret.getFaults());
                        turretCommands.addString("To Jog", () -> "SetupXBox Btn A left X");
                        turretCommands.addString("OvrRideSoftLim", () -> "Setup RightBmpr");

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
                        turretValues.addNumber("AdjTarget", () -> m_turret.adjustedCameraError);
                        turretValues.addNumber("Vision Error", () -> m_limelight.getdegRotationToTarget());
                        turretValues.addNumber("DriverOffset", () -> m_turret.driverHorizontalOffsetDegrees);
                        turretValues.addNumber("LockPosnErr", () -> m_turret.getLockPositionError());

                        ShuffleboardLayout turretValues3 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        turretValues3.addNumber("IAccum", () -> m_turret.getIaccum());

                        turretValues3.addNumber("LockOutput", () -> m_turret.lockPIDOut);
                        turretValues3.addNumber("LockError", () -> m_turret.m_turretLockController.getPositionError());
                        turretValues3.addBoolean("LockController", () -> m_turret.validTargetSeen);

                        ShuffleboardLayout turretValues2 = Shuffleboard.getTab("SetupTurret")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(2, 3).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "TOP")); // labels
                        turretValues2.addBoolean("Connected (8)", () -> m_turret.turretMotorConnected);

                        turretValues2.addBoolean("PlusLimit", () -> m_turret.onPlusSoftwareLimit());

                        turretValues2.addBoolean("MinusLimit", () -> m_turret.onMinusSoftwareLimit());

                        turretValues2.addBoolean("SWLimitEn", () -> m_turret.getSoftwareLimitsEnabled());

                        turretValues2.addBoolean("InPosition", () -> m_turret.atTargetAngle());

                        turretValues2.addBoolean("BrakeMode", () -> m_turret.isBrake());
                        turretValues2.addBoolean("TargetHorOK",
                                        () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));

                        turretValues2.addBoolean("OKTune", () -> (m_turret.tuneOnv && m_turret.lastTuneOnv));

                        turretValues2.addBoolean("LockAtTarget", () -> m_turret.getLockAtTarget());

                        ShuffleboardLayout turretGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        turretGains.addNumber("FF", () -> m_turret.ffset);
                        turretGains.addNumber("P", () -> m_turret.pset);
                        turretGains.addNumber("I", () -> m_turret.iset);
                        turretGains.addNumber("D", () -> m_turret.dset);
                        turretGains.addNumber("IZ", () -> m_turret.izset);
                        turretGains.addNumber("MaxAcc", () -> m_turret.maxAccset);
                        turretGains.addNumber("MaxV", () -> m_turret.maxVelset);

                        ShuffleboardLayout turretVGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("VelGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        turretVGains.addNumber("FF", () -> m_turret.ffsetv);
                        turretVGains.addNumber("P", () -> m_turret.psetv);
                        turretVGains.addNumber("I", () -> m_turret.isetv);
                        turretVGains.addNumber("D", () -> m_turret.dsetv);
                        turretVGains.addNumber("IZ", () -> m_turret.izsetv);

                        ShuffleboardLayout turretLockGains = Shuffleboard.getTab("SetupTurret")

                                        .getLayout("LockGains", BuiltInLayouts.kList).withPosition(6, 1).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels
                        turretLockGains.addNumber("LP", () -> m_turret.lpset);
                        turretLockGains.addNumber("LI", () -> m_turret.liset);
                        turretLockGains.addNumber("LD", () -> m_turret.ldset);
                        turretGains.addNumber("LIZ", () -> m_turret.lizset);
                }
                /**
                 * Shooter Tilt
                 */
                if (m_showTilt && !liveMatch) {
                        ShuffleboardLayout tiltCommands = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("Tilt", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); //

                        tiltCommands.add("Position To 25", new PositionTilt(m_tilt, 25));
                        tiltCommands.add("Position To 15", new PositionTilt(m_tilt, 15));
                        tiltCommands.add("Position To 5", new PositionTilt(m_tilt, 5));
                        tiltCommands.add("PositionToVision", new PositionTiltToVision(m_tilt, m_limelight, 10));
                        // HoodedShooterConstants.TILT_MIN_ANGLE));
                        tiltCommands.add(new PositionTiltToVision(m_tilt, m_limelight, m_tilt.tiltMinAngle));
                        tiltCommands.add("PositionToSwitch", new TiltMoveToReverseLimit(m_tilt));
                        tiltCommands.add("StopTilt", new StopTilt(m_tilt));
                        tiltCommands.add("ClearFaults", new ClearFaults(m_tilt));
                        tiltCommands.add("Cmd", m_tilt);
                        tiltCommands.addNumber("Faults", () -> m_tilt.faultSeen);

                        ShuffleboardLayout tiltValues = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("TiltValues", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues.addNumber("TICameraAngle", () -> m_tilt.getAngle());
                        tiltValues.addNumber("TITgt", () -> m_tilt.targetAngle);
                        tiltValues.addNumber("PCT", () -> m_tilt.getOut());
                        tiltValues.addNumber("Amps", () -> m_tilt.getAmps());
                        tiltValues.addNumber("Speed", () -> m_tilt.getSpeed());
                        tiltValues.addNumber("Vision Offset", () -> m_tilt.targetVerticalOffset);
                        tiltValues.addNumber("AdjTarget", () -> m_tilt.adjustedVerticalError);
                        tiltValues.addNumber("Vision Error", () -> m_limelight.getdegVerticalToTarget());
                        tiltValues.addNumber("MotorDeg", () -> m_tilt.getMotorDegrees());
                        tiltValues.addNumber("MotorTarget", () -> m_tilt.motorEndpointDegrees);
                        tiltValues.addNumber("DriverOffset", () -> m_tilt.driverVerticalOffsetDegrees);
                        tiltValues.addNumber("LockError", () -> m_tilt.tiltLockController.getPositionError());

                        ShuffleboardLayout tiltValues3 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("PIDValues", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for

                        tiltValues3.addNumber("IAccum", () -> m_tilt.getIaccum());

                        tiltValues3.addNumber("LockOutput", () -> m_tilt.lockPIDOut);
                        tiltValues3.addNumber("LockError", () -> m_tilt.tiltLockController.getPositionError());
                        tiltValues3.addBoolean(("LockController"), () -> m_tilt.validTargetSeen);
                        tiltValues3.addBoolean("LockOnTarget", () -> m_tilt.getLockAtTarget());
                        tiltValues3.addBoolean("ValTgt", () -> m_tilt.validTargetSeen);

                        ShuffleboardLayout tiltValues2 = Shuffleboard.getTab("SetupTilt")
                                        .getLayout("States", BuiltInLayouts.kGrid).withPosition(4, 2).withSize(3, 2)
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
                        tiltValues2.addBoolean("TargetVertOK",
                                        () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));

                        ShuffleboardLayout tiltGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("MMGains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        tiltGains.addNumber("FF", () -> m_tilt.ffset);
                        tiltGains.addNumber("P", () -> m_tilt.pset);
                        tiltGains.addNumber("I", () -> m_tilt.iset);
                        tiltGains.addNumber("D", () -> m_tilt.dset);
                        tiltGains.addNumber("IZ", () -> m_tilt.izset);
                        tiltGains.addNumber("MaxAcc", () -> m_tilt.maxAccset);
                        tiltGains.addNumber("MaxV", () -> m_tilt.maxVelset);
                        ShuffleboardLayout tiltVGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("VelGains", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        tiltVGains.addNumber("FF", () -> m_tilt.ffsetv);
                        tiltVGains.addNumber("P", () -> m_tilt.psetv);
                        tiltVGains.addNumber("I", () -> m_tilt.isetv);
                        tiltVGains.addNumber("D", () -> m_tilt.dsetv);
                        tiltVGains.addNumber("IZ", () -> m_tilt.izsetv);

                        ShuffleboardLayout tiltLockGains = Shuffleboard.getTab("SetupTilt")

                                        .getLayout("LockGains", BuiltInLayouts.kList).withPosition(6, 1).withSize(1, 1)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        tiltLockGains.addNumber("LP", () -> m_tilt.lpset);
                        tiltLockGains.addNumber("LI", () -> m_tilt.liset);
                        tiltLockGains.addNumber("LD", () -> m_tilt.ldset);
                        tiltLockGains.addNumber("LIZ", () -> m_tilt.lizset);

                }
                /**
                 * 
                 * Shooter and Transport
                 * 
                 */
                if (m_showShooter && !liveMatch)

                {
                        ShuffleboardLayout shooterCommands = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("MAXMPS 50", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 3)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels for
                                                                                           // commands

                        shooterCommands.add("Stop Shoot", new StopShoot(m_shooter, m_transport));
                        shooterCommands.add("Shoot", new ShootCells(m_shooter, m_tilt, m_turret, m_limelight,
                                        m_transport, drive, m_compressor, 0));
                        shooterCommands.add("ClearFaults", new ClearShFaults(m_shooter));
                        shooterCommands.add("Cmd", m_shooter);
                        shooterCommands.add("LogDataRun",
                                        new LogDistanceData(m_robotDrive, m_turret, m_tilt, m_shooter, m_limelight));
                        shooterCommands.add("RunAllShooters", new RunShooter(shooter));
                        shooterCommands.add("UseSpeedSlider", new ChooseShooterSpeedSource(shooter, tilt, turret, 3));

                        ShuffleboardLayout shooterValues = Shuffleboard.getTab("SetupShooter")
                                        .getLayout("ShooterValues", BuiltInLayouts.kList).withPosition(2, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels
                                                                                                          // for

                        shooterValues.addNumber("LeftMPS", () -> m_shooter.getMPS());
                        shooterValues.addNumber("Left PCT", () -> m_shooter.getLeftPctOut());
                        shooterValues.addNumber("LeftAmps", () -> m_shooter.getLeftAmps());
                        shooterValues.addNumber("RightRPM", () -> m_shooter.getRightRPM());
                        shooterValues.addNumber("RightAmps", () -> m_shooter.getRightAmps());
                        shooterValues.addNumber("SpeedCommand MPS", () -> m_shooter.requiredMps);
                        shooterValues.addNumber("LeftFaults", () -> m_shooter.getLeftFaults());
                        shooterValues.addNumber("RightFaults", () -> m_shooter.getRightFaults());
                        shooterValues.addBoolean("CameraHasSpeed", () -> m_shooter.useCameraSpeed);

                        ShuffleboardLayout shooterValues1 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("ShooterValues1", BuiltInLayouts.kList).withPosition(4, 0)
                                        .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues1.addNumber("VertOffset", () -> m_tilt.targetVerticalOffset);
                        shooterValues1.addNumber("HorOffset", () -> m_turret.targetHorizontalOffset);
                        shooterValues1.addNumber("DriverVOffset", () -> m_tilt.driverVerticalOffsetDegrees);
                        shooterValues1.addNumber("DriverHOffset", () -> m_turret.driverHorizontalOffsetDegrees);

                        shooterValues1.addNumber("TargetDistance", () -> m_shooter.calculatedCameraDistance);
                        shooterValues1.addNumber("CameraAngle", () -> m_tilt.getCameraAngle());

                        shooterValues1.addBoolean("AtSpeed", () -> m_shooter.atSpeed());
                        shooterValues1.addBoolean("TuneOn", () -> (m_shooter.tuneOn && m_shooter.lastTuneOn));
                        shooterValues1.addBoolean("BothConnected (6,7)", () -> m_shooter.allConnected);
                        shooterValues1.addBoolean("DriverOKShoot", () -> m_shooter.driverOKShoot);
                        shooterValues1.addBoolean("ShootOne", () -> m_shooter.shootOne);
                        shooterValues1.addBoolean("UsingSliders", () -> m_shooter.useSetupSlider);

                        ShuffleboardLayout shooterValues2 = Shuffleboard.getTab("SetupShooter")

                                        .getLayout("Gains", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2)
                                        .withProperties(Map.of("Label position", "LEFT")); // labels

                        shooterValues2.addNumber("FF", () -> m_shooter.ffset);
                        shooterValues2.addNumber("P", () -> m_shooter.pset);
                        shooterValues2.addNumber("I", () -> m_shooter.iset);
                        shooterValues2.addNumber("D", () -> m_shooter.dset);
                        shooterValues2.addNumber("IZ", () -> m_shooter.izset);

                        m_shooter.shooterSpeed = Shuffleboard.getTab("SetupShooter").add("ShooterSpeed", 3)
                                        .withWidget("Number Slider").withPosition(0, 3).withSize(4, 1)
                                        .withProperties(Map.of("Min", 15, "Max", 50)).getEntry();

                }

                if (m_showTransport && !liveMatch) {
                        ShuffleboardLayout transportValues = Shuffleboard.getTab("SetupTransport")
                                        .getLayout("TransportValues", BuiltInLayouts.kList).withPosition(0, 0)
                                        .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                        transportValues.add("StartRollers", new RunRollers(transport));
                        transportValues.add("StopRollers", new StopRollers(transport));

                        transportValues.addNumber("LeftBeltAmps", () -> m_transport.getLeftBeltMotorAmps());
                        transportValues.addNumber("RightBeltAmps", () -> m_transport.getRightBeltMotorAmps());
                        transportValues.addNumber("FrontRollerAmps", () -> m_transport.getFrontRollerMotorAmps());
                        transportValues.addNumber("RearRollerAmps", () -> m_transport.getRearRollerMotorAmps());
                        transportValues.addNumber("LeftBeltOut", () -> m_transport.getLeftBelt());
                        transportValues.addNumber("RighBeltOut", () -> m_transport.getRightBelt());
                        transportValues.addNumber("FrontRollerOut", () -> m_transport.getFrontRoller());
                        transportValues.addNumber("RearRollerOut", () -> m_transport.getRearRoller());
                        transportValues.add("Cmd", m_transport);

                        if (m_showTransport && !liveMatch) {
                                ShuffleboardLayout transportCellArm = Shuffleboard.getTab("SetupTransport")
                                                .getLayout("TransportCellArm", BuiltInLayouts.kList).withPosition(2, 0)
                                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT")); // labels

                                transportCellArm.add("Hold Cell", new HoldCell(transport));
                                transportCellArm.add("ReleaseOneCell", new ReleaseOneCell(transport));
                                transportCellArm.addNumber(("CellArmAngle"), () -> m_transport.getArmAngle());
                                transportCellArm.addNumber(("CellArmPosn"), () -> m_transport.getArmPosition());
                                // for
                                transportCellArm.add("Release Cell", new ReleaseCell(transport));

                                transportCellArm.add("RaiseLeft", new RaiseLeftArm(transport));
                                transportCellArm.add("LowerLeft", new LowerLeftArm(transport));

                                transportCellArm.addBoolean("Arm Up", () -> m_transport.getCellArmUp());
                                transportCellArm.addBoolean("Arm Down", () -> m_transport.getCellArmDown());
                                transportCellArm.addBoolean("Left Arm Up", () -> m_transport.getLeftArmUp());
                                transportCellArm.addBoolean("Left Arm Down", () -> m_transport.getLeftArmDown());

                                ShuffleboardLayout transportValues1 = Shuffleboard.getTab("SetupTransport")
                                                .getLayout("TransportStates", BuiltInLayouts.kGrid).withPosition(4, 0)
                                                .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // label

                                transportValues1.addBoolean("IntakeConnected (10)",
                                                () -> m_intake.intakeMotorConnected);
                                transportValues1.addBoolean("RollersAtSpeed", () -> m_transport.rollersAtSpeed);

                                transportValues1.addBoolean("LeftBeltConnected (13)",
                                                () -> m_transport.leftBeltMotorConnected);
                                transportValues1.addBoolean("RightBeltConnected (11)",
                                                () -> m_transport.rightBeltMotorConnected);
                                transportValues1.addBoolean("RearRollerConnected (12)",
                                                () -> m_transport.rearRollerMotorConnected);
                                transportValues1.addBoolean("FrontRollerConnected (14)",
                                                () -> m_transport.frontRollerMotorConnected);
                        }

                        /**
                         * 
                         * Robot
                         * 
                         */
                        if (m_showRobot & !liveMatch)

                        {
                                ShuffleboardLayout robotCommands = Shuffleboard.getTab("SetupRobot")
                                                .getLayout("Robot", BuiltInLayouts.kList).withPosition(0, 0)
                                                .withSize(2, 4).withProperties(Map.of("Label position", "LEFT"));

                                robotCommands.add("Reset Enc", new ResetEncoders(m_robotDrive));
                                robotCommands.add("Reset Gyro", new ResetGyro(m_robotDrive));

                                robotCommands.add("ClearFaults", new ClearRobFaults(m_robotDrive));
                                robotCommands.add("Stop Robot", new StopRobot(m_robotDrive));
                                robotCommands.add("To -4(2)", new PickupMoveVelocity(m_robotDrive, -4, 2));
                                robotCommands.add("To -4(3)", new PickupMoveVelocity(m_robotDrive, -4, 3));
                                robotCommands.add("To -4(1)", new PickupMoveVelocity(m_robotDrive, -4, 1));
                                robotCommands.add("To 0(1)", new PickupMoveVelocity(m_robotDrive, 0, 1));
                                robotCommands.add("To 0", new PickupMove(m_robotDrive, 0, .5));
                                robotCommands.add("Cmd", m_robotDrive);

                                robotCommands.add("EndLog", new EndDriveLog(m_robotDrive));

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
                                                .getLayout("States", BuiltInLayouts.kGrid).withPosition(5, 0)
                                                .withSize(2, 4).withProperties(Map.of("Label position", "TOP")); // labels

                                robotValues2.addBoolean("TuneOn",
                                                () -> (m_robotDrive.tuneOn && m_robotDrive.lastTuneOn));
                                robotValues2.addBoolean("Left1Connected (2)", () -> m_robotDrive.leftLeadConnected);
                                robotValues2.addBoolean("Left2Connected (3)", () -> m_robotDrive.leftFollowerConnected);
                                robotValues2.addBoolean("Right1Connected (4)", () -> m_robotDrive.rightLeadConnected);
                                robotValues2.addBoolean("Right2Connected (5)",
                                                () -> m_robotDrive.rightFollowerConnected);
                                robotValues2.addBoolean("LInPosition", () -> m_robotDrive.getInPositionLeft());
                                robotValues2.addBoolean("RInPosition", () -> m_robotDrive.getInPositionRight());
                                robotValues2.addBoolean("LFoll", () -> m_robotDrive.getLeftFollower());
                                robotValues2.addBoolean("RFoll", () -> m_robotDrive.getRightFollower());

                                ShuffleboardLayout robotGains = Shuffleboard.getTab("SetupRobot")
                                                .getLayout("Gains", BuiltInLayouts.kGrid).withPosition(6, 0)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); // labels

                                robotGains.addNumber("LFF", () -> m_robotDrive.ffset);
                                robotGains.addNumber("LP", () -> m_robotDrive.pset);
                                robotGains.addNumber("RFF", () -> m_robotDrive.rffset);
                                robotGains.addNumber("RP", () -> m_robotDrive.rpset);

                        }
                        /**
                         * 
                         * Subsystems
                         * 
                         */

                        if (m_showSubsystems && !liveMatch) {
                                ShuffleboardLayout subSystems = Shuffleboard.getTab("Can+Sols")
                                                .getLayout("All", BuiltInLayouts.kList).withPosition(0, 0)
                                                .withSize(3, 7).withProperties(Map.of("Label position", "LEFT")); //
                                                                                                                  // labels
                                                                                                                  // for
                                subSystems.add("Drive", m_robotDrive);
                                subSystems.add("Shooter", m_shooter);
                                subSystems.add("Turret", m_turret);
                                subSystems.add("Tilt", m_tilt);
                                subSystems.add("Transport", m_transport);
                                subSystems.add("Intake", m_intake);

                                ShuffleboardLayout scheduler = Shuffleboard.getTab("Can+Sols")
                                                .getLayout("Scheduler", BuiltInLayouts.kList).withPosition(3, 0)
                                                .withSize(7, 2).withProperties(Map.of("Label position", "TOP")); //

                                scheduler.add("Scheduler", CommandScheduler.getInstance());

                                ShuffleboardLayout canBus = Shuffleboard.getTab("Pre-Round")
                                                .getLayout("Canbus", BuiltInLayouts.kGrid).withPosition(3, 2)
                                                .withSize(4, 2).withProperties(Map.of("Label position", "TOP")); // labels

                                canBus.addBoolean("TurretConnected (8)", () -> m_turret.turretMotorConnected);
                                canBus.addBoolean("TiltConnected (9)", () -> m_tilt.tiltMotorConnected);
                                canBus.addBoolean("LeftShooterConnected (6)", () -> m_shooter.leftMotorConnected);
                                canBus.addBoolean("RightShooterConnected (7)", () -> m_shooter.rightMotorConnected);
                                canBus.addBoolean("LeftBeltConnected (13)", () -> m_transport.leftBeltMotorConnected);
                                canBus.addBoolean("RightBeltConnected (11)", () -> m_transport.rightBeltMotorConnected);
                                canBus.addBoolean("RearRollerConnected (12)",
                                                () -> m_transport.rearRollerMotorConnected);
                                canBus.addBoolean("FrontRollerConnected (14)",
                                                () -> m_transport.frontRollerMotorConnected);
                                canBus.addBoolean("LDR1Connected (2)", () -> m_robotDrive.leftLeadConnected);
                                canBus.addBoolean("LDr2Connected (3)", () -> m_robotDrive.leftFollowerConnected);
                                canBus.addBoolean("RDr1Connected (4)", () -> m_robotDrive.rightLeadConnected);
                                canBus.addBoolean("RDr2Connected (5)", () -> m_robotDrive.rightFollowerConnected);
                                canBus.addBoolean("IntakeConnected (10)", () -> m_intake.intakeMotorConnected);

                                ShuffleboardLayout sols = Shuffleboard.getTab("Can+Sols")
                                                .getLayout("Solenoids", BuiltInLayouts.kGrid).withPosition(7, 2)
                                                .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // labels

                                sols.addString("Intake Sol", () -> " pins 2 and 3");

                        }
                        /**
                         * 
                         * Vision
                         * 
                         */
                        if (m_showVision) {
                                ShuffleboardLayout zoomCommands = Shuffleboard.getTab("Vision")
                                                .getLayout("Zoom", BuiltInLayouts.kList).withPosition(0, 0)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //
                                                                                                                 // labels
                                                                                                                 // for

                                zoomCommands.add("No Zoom", new LimelightSetPipeline(m_limelight, 1));
                                zoomCommands.add("2XZoom", new LimelightSetPipeline(m_limelight, 2));
                                zoomCommands.add("3X Zoom", new LimelightSetPipeline(m_limelight, 3));

                                ShuffleboardLayout visionCommands = Shuffleboard.getTab("Vision")
                                                .getLayout("On-Off", BuiltInLayouts.kList).withPosition(0, 2)
                                                .withSize(1, 2).withProperties(Map.of("Label position", "TOP")); //

                                visionCommands.add("Vision On", new UseVision(limelight, true));
                                visionCommands.add("Vision Off", new UseVision(limelight, false));

                                ShuffleboardLayout cameraCommands = Shuffleboard.getTab("Vision")
                                                .getLayout("Camera", BuiltInLayouts.kList).withPosition(1, 0)
                                                .withSize(1, 4).withProperties(Map.of("Label position", "TOP")); // labels
                                                                                                                 // for

                                cameraCommands.add("DriverCam", new LimelightCamMode(m_limelight, CamMode.kdriver));
                                cameraCommands.add("VisionCam", new LimelightCamMode(m_limelight, CamMode.kvision));

                                cameraCommands.add("SideBySideStream",
                                                new LimelightStreamMode(m_limelight, StreamType.kStandard));
                                cameraCommands.add("MainPIP",
                                                new LimelightStreamMode(m_limelight, StreamType.kPiPMain));
                                cameraCommands.add("SecIP",
                                                new LimelightStreamMode(m_limelight, StreamType.kPiPSecondary));

                                ShuffleboardLayout ledCommands = Shuffleboard.getTab("Vision")
                                                .getLayout("LEDs", BuiltInLayouts.kList).withPosition(2, 0)
                                                .withSize(1, 3).withProperties(Map.of("Label position", "TOP"));

                                ledCommands.add("LedsOn", new LimelightLeds(m_limelight, LedMode.kforceOn));
                                ledCommands.add("LedsOff", new LimelightLeds(m_limelight, LedMode.kforceOff));
                                ledCommands.add("LedsBlink", new LimelightLeds(m_limelight, LedMode.kforceBlink));
                                ledCommands.add("LedsPipeline", new LimelightLeds(m_limelight, LedMode.kpipeLine));

                                ShuffleboardLayout visionData = Shuffleboard.getTab("Vision")
                                                .getLayout("Data", BuiltInLayouts.kList).withPosition(3, 0)
                                                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT")); //

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
                                                .getLayout("States", BuiltInLayouts.kGrid).withPosition(3, 3)
                                                .withSize(2, 2).withProperties(Map.of("Label position", "TOP")); // labels

                                visionBools.addBoolean("Connected", () -> m_limelight.isConnected());

                                visionBools.addBoolean("TargetVertOK",
                                                () -> m_limelight.getVertOnTarget(m_tilt.tiltVisionTolerance));

                                visionBools.addBoolean("TargetHorOK",
                                                () -> m_limelight.getHorOnTarget(m_turret.turretVisionTolerance));

                                visionBools.addBoolean("TargetFound", () -> m_limelight.getIsTargetFound());

                                visionBools.addBoolean("Use Vision", () -> m_limelight.useVision);

                                if (RobotBase.isReal()) {

                                        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
                                        ShuffleboardTab driverDisplayTab = Shuffleboard.getTab("Vision");
                                        driverDisplayTab.add("Limelight", LLFeed)
                                                        .withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 0)
                                                        .withSize(5, 4).withProperties(Map.of("Show Crosshair", true,
                                                                        "Show Controls", false));// specify
                                                                                                 // widget
                                                                                                 // properties
                                                                                                 // here
                                }
                        }
                }

        }

        public void checkCANDevices() {
                m_turret.checkCAN();
                m_tilt.checkCAN();
                m_intake.checkCAN();
                m_shooter.checkCAN();
                m_robotDrive.checkCAN();
                m_transport.checkCAN();

        }

        public double[] getPDPInfo() {
                double temp[] = { 0, 0, 0, 0, 0 };
                temp[0] = m_shooter.getBatteryVoltage();
                temp[1] = m_shooter.getTemperature();
                temp[2] = m_shooter.getTotalEnergy() / 3600;
                temp[3] = m_shooter.getTotalPower();
                return temp;

        }

        public void checkLimits() {
                if (m_tilt.onMinusSoftwareLimit() || m_tilt.onPlusSoftwareLimit() || m_tilt.onMinusHardwarLimit()
                                || m_turret.onPlusSoftwareLimit() || m_turret.onMinusSoftwareLimit()
                                || DriverStation.getInstance().isDisabled())
                        m_limelight.useVision = false;

        }

}
