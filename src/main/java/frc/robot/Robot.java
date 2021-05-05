/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RobotDrive.PositionRobot;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Vision.AutoSwitchZoom;
import frc.robot.commands.Vision.CalculateTargetDistance;

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

  public RobotContainer m_robotContainer;
  private boolean autoHasRun;
  private double m_startDelay;
  private double startTime;
  public double timeToStart;
  public POV driverPOV;
  public POVXBox gamepadPOV;
  public POVBBox boxPOV;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings.

    m_robotContainer = new RobotContainer();

    driverPOV = new POV(m_robotContainer.m_driverController);
    gamepadPOV = new POVXBox(m_robotContainer.m_gamepad);
    boxPOV = new POVBBox(m_robotContainer.buttonBox);
    Shuffleboard.selectTab("Pre-Round");

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
    SmartDashboard.updateValues();
    // CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().run();
    boolean canCheckOK = m_robotContainer.m_setup.checkCANDevices();
  }

  @Override
  public void disabledPeriodic() {

  }

  }

  public void autonomousInit() {

     new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    AutoFactory m_autoFactory = m_robotContainer.m_autoFactory;

    Shuffleboard.selectTab("Competition");

    // get delay time

    m_startDelay = (double) m_robotContainer.m_setup.startDelayChooser.getSelected();

    // SmartDashboard.putNumber("Delay", m_startDelay);

    autoChoice = m_robotContainer.m_setup.autoChooser.getSelected();

    switch (autoChoice) {

    case 0:// in front of power port use 0 shooter data index use pipeline 0 - no zoom
      setStartingPose(
          new Pose2d(FieldMap.startLineX - FieldMap.robotLength, FieldMap.targetCenterPointY, new Rotation2d(0)));
      m_autoFactory.shootNumber = 0;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
      break;
    case 1:// in front of power port, move back use 1 shooter data index use pipeline 0 -
           // no zoom

      m_autoFactory.shootNumber = 1;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
      break;

    case 2:// Left of power port use 2 shooter data index use pipeline 0 - no zoom

      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.targetCenterPointY + FieldMap.robotWidth + .25, new Rotation2d(0)));
      m_autoFactory.shootNumber = 2;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();

      break;

    case 3:// Left of power port move back use 3 shooter data index use pipeline 0 - no
           // zoom
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.targetCenterPointY + FieldMap.robotWidth + .25, new Rotation2d(0)));

      m_autoFactory.shootNumber = 3;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
      break;

    case 4:// Right of power port use 4 shooter data index use pipeline 0 - no zoom
           // m_autoFactory.shootNumber = 0;
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.targetCenterPointY - FieldMap.robotWidth - .25, new Rotation2d(0)));
      m_autoFactory.shootNumber = 4;

      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
      break;

    case 5:// Right of power port nmove back use 5 shooter data index use pipeline 0 - no
           // zoom
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.targetCenterPointY - FieldMap.robotWidth - .25, new Rotation2d(0)));
      m_autoFactory.shootNumber = 5;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
      break;

    case 6:// Front of trench move back pick up 2 shoot use 6 shooter data index use
           // pipeline 0 - no zoom
      // move back pick up 3, return shoot
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.friendlyTrenchY + FieldMap.robotWidth / 2, new Rotation2d(0)));
      m_autoFactory.shootNumber = 6;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand1();
      break;

    case 7:// Front of trench move back use 6 shooter data index move back again pickup and
           // use 7 shooter data
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.friendlyTrenchY + FieldMap.robotWidth / 2, new Rotation2d(0)));
      m_autoFactory.shootNumber = 6;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand2();
      break;
    case 8:// Front of trench move back use 6 shooter data index move back under control
           // panel pickup move back and shoot
      // use 7 shooter data
      setStartingPose(new Pose2d(FieldMap.startLineX - FieldMap.robotLength,
          FieldMap.friendlyTrenchY + FieldMap.robotWidth / 2, new Rotation2d(0)));
      m_autoFactory.shootNumber = 6;
      m_autonomousCommand = m_autoFactory.getAutonomousCommand3();
      break;

    case 9:// cross line

      setStartingPose(
          new Pose2d(FieldMap.startLineX - FieldMap.robotLength, FieldMap.fieldWidth - 2, new Rotation2d(0)));
      m_autonomousCommand = new PositionRobot(m_robotContainer.m_robotDrive, -1, 5);

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
    autoHasRun = false;
    // if (!m_robotContainer.m_tilt.positionResetDone)
    // new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);
    // CommandScheduler.getInstance().cancelAll();

    new CalculateTargetDistance(m_robotContainer.m_limelight, m_robotContainer.m_tilt, m_robotContainer.m_shooter)
        .schedule(true);
    new AutoSwitchZoom(m_robotContainer.m_limelight).schedule(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();

    if (driverPOV.DPadUp() || boxPOV.DPadUp())
      m_robotContainer.m_tilt.aimHigher(.25);
    if (driverPOV.DPadDown() || boxPOV.DPadDown())
      m_robotContainer.m_tilt.aimLower(.25);

    if (driverPOV.DPadLeft() || boxPOV.DPadLeft())
      m_robotContainer.m_turret.aimFurtherLeft(.25);
    if (driverPOV.DPadRight() || boxPOV.DPadRight())
      m_robotContainer.m_turret.aimFurtherRight(.25);

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

  private void setStartingPose(Pose2d pose) {

    m_robotContainer.m_robotDrive.resetAll();

    if (RobotBase.isSimulation())
      m_robotContainer.m_robotDrive.fieldSim.setRobotPose(pose);

    m_robotContainer.m_robotDrive.resetPose(pose);
  }

}
