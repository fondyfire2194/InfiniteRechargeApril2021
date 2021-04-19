/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int autoChoice;

  private RobotContainer m_robotContainer;
  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;
  public double timeToStart;
  private Translation2d startPosition;
  final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
  final Rotation2d rotation270 = Rotation2d.fromDegrees(270.0);
  final Rotation2d rotation90 = Rotation2d.fromDegrees(90.0);

  public static NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
  public static edu.wpi.first.networktables.NetworkTable tuneValues;
  public NetworkTableEntry kpEntry;
  public NetworkTableEntry kiEntry;
  public NetworkTableEntry kiZEntry;
  public NetworkTableEntry kdEntry;
  public NetworkTableEntry kFFEntry;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings.

    m_robotContainer = new RobotContainer();

    Shuffleboard.selectTab("Pre-Round");
    ntinst.startServer();
    tuneValues = ntinst.getTable("tuneValues");

    kpEntry = tuneValues.getEntry("kP");
    kiEntry = tuneValues.getEntry("kI");
    kiZEntry = tuneValues.getEntry("kIZ");
    kdEntry = tuneValues.getEntry("kD");
    kFFEntry = tuneValues.getEntry("kFF");

    kpEntry.setDouble(.002);
    kiEntry.setDouble(0.01);
    kiZEntry.setDouble(2);
    kdEntry.setDouble(0);
    kFFEntry.setDouble(2e-4);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {

    // CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  public void autonomousInit() {
    new TiltMoveToReverseLimit(m_robotContainer.m_tilt);
    m_robotContainer.m_turret.setDefaultCommand(new PositionHoldTurret(m_robotContainer.m_turret));
    FondyFireTrajectory m_trajectory = m_robotContainer.m_trajectory;
    RevDrivetrain m_robotDrive = m_robotContainer.m_robotDrive;

    Shuffleboard.selectTab("Competition");

    // get delay time

    m_startDelay = (double) m_robotContainer.m_setup.startDelayChooser.getSelected();

    SmartDashboard.putNumber("Delay", m_startDelay);

    autoChoice = m_robotContainer.m_setup.autoChooser.getSelected();

    switch (autoChoice) {

    case 0:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());

      m_robotDrive.resetAll();
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand0();

      break;

    case 1:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand1();

      break;

    case 2:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand2();

      break;

    case 3:// in front of power port 0 shooter data index use pipeline 0 - no zoom
      SmartDashboard.putBoolean("AUTO3", true);
      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand3();

      break;

    case 4:// cross line
      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      new PositionRobot(m_robotDrive, -2).schedule(true);
      break;

    case 5:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.centerStart.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.centerStart.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand4();

      break;

    case 7:// in front of power port 0 shooter data index use pipeline 0 - no zoom

      m_robotDrive.resetAll();
      m_robotDrive.resetOdometry(m_trajectory.example.getInitialPose());
      if (RobotBase.isSimulation())
        m_robotDrive.fieldSim.setRobotPose(m_trajectory.example.getInitialPose());
      m_autonomousCommand = m_robotContainer.getAutonomousCommand4();

      break;

    default:

      break;

    }
    startTime = Timer.getFPGATimestamp();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }

    m_robotContainer.m_setup.timeToStart = Math.round(startTime + m_startDelay - Timer.getFPGATimestamp());

    if (m_robotContainer.m_setup.timeToStart < 0)
      m_robotContainer.m_setup.timeToStart = 0;

    // CommandScheduler.getInstance().run();

    if (DriverStation.getInstance().getMatchTime() < 10)
      Shuffleboard.stopRecording();

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);
    // CommandScheduler.getInstance().cancelAll();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

}
