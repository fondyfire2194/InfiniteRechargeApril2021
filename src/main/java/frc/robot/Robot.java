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

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings.

    m_robotContainer = new RobotContainer();

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
    m_robotContainer.m_setup.checkCANDevices();
    ShootData.showValues(1);
    ShootData.showValues(2);
    ShootData.showValues(3);

  }

  @Override
  public void disabledPeriodic() {

  }

  public void autonomousInit() {

    if (RobotBase.isReal())
      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);

    m_robotContainer.m_turret.enableSofLimits(true);

    AutoFactory m_autoFactory = m_robotContainer.m_autoFactory;

    Shuffleboard.selectTab("Competition");

    // get delay time

    m_startDelay = (double) m_robotContainer.m_setup.startDelayChooser.getSelected();

    autoChoice = m_robotContainer.m_setup.autoChooser.getSelected();
    m_robotContainer.m_limelight.useVision = true;
   SmartDashboard.putNumber("ASel", autoChoice); 
    switch (autoChoice) {

      case 0:// cross line

        setStartingPose(FieldMap.startPosition[0]);
        m_robotContainer.m_limelight.useVision = false;
        m_autonomousCommand = new PositionRobot(m_robotContainer.m_robotDrive, -1);

        break;
      case 1:// in front of power port, move back use shooter data index 1

        setStartingPose(FieldMap.startPosition[1]);
        m_autoFactory.shootNumber = ShootData.retractOneStraight;
        m_autonomousCommand = m_autoFactory.getAutonomousCommand0();
        break;

      case 2:// Left of power port use 1 shoot data

        setStartingPose(FieldMap.startPosition[2]);
        m_autoFactory.shootNumber = ShootData.leftTwoBall;
        m_autonomousCommand = m_autoFactory.getAutonomousCommand1();

        break;

      case 3:// Trench
             
        setStartingPose(FieldMap.startPosition[3]);
        m_autoFactory.shootNumber = ShootData.trenchTwoBall;

        m_autonomousCommand = m_autoFactory.getAutonomousCommand1();
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
    if (RobotBase.isReal() && !m_robotContainer.m_tilt.positionResetDone)
      new TiltMoveToReverseLimit(m_robotContainer.m_tilt).schedule(true);
    // CommandScheduler.getInstance().cancelAll();

    new CalculateTargetDistance(m_robotContainer.m_limelight, m_robotContainer.m_tilt, m_robotContainer.m_shooter)
        .schedule(true);
    new AutoSwitchZoom(m_robotContainer.m_limelight).schedule(true);

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override

  /**
   * In teleop after cells have been picked up, the tilt and turret need to be
   * pointed at the target so the Limelight can be used to lock them on.
   * 
   * The robot is designed so it can use the trench and this will be the default
   * turret / tilt positions afer a pickup. Thes axes will be positioned to a
   * place where the camera can pick up the target as the robot comes out from
   * under the control panel. At this point they will both be in position hold
   * mode until a target is seen and then they will move to lock on the target.
   * Target horizontal and vertical offsets can be used to shift the lock on
   * points. To lock on with vision a limelight useVision boolean must be set. A
   * Shoot when Ready command can be started at any point and will initiate
   * shooting when the shooter is at speed, the tilt and turret are lock on and
   * the robot is not moving. Shooting will prevent the joystick being used to
   * move the robot.
   * 
   * If the trench is not being used, the co driver picks either a right, left,
   * short straight or long straight shot turret from driver instructions. The
   * tilt angle will be move from its base 30 degree angle towards 0. It should
   * then pick up a target and both tilt and turret will lock on.
   * 
   * Shot speed is determined from distance calculated from target height and
   * camera angle. Speeds need to be empirically determined and put in a table to
   * be interpolated.
   * 
   */
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    // m_robotContainer.setupGamepad.setRumble(RumbleType.kLeftRumble, 1.0);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
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
