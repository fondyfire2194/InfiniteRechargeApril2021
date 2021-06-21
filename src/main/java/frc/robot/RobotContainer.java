/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.CellIntake.StartIntake;
import frc.robot.commands.CellIntake.StopIntake;
import frc.robot.commands.CellTransport.JogLeftBelt;
import frc.robot.commands.CellTransport.JogRightBelt;
import frc.robot.commands.CellTransport.ReleaseOneCell;

import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.LockAndShoot;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetActiveTeleopShootData;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.TiltJog;
import frc.robot.commands.Tilt.TiltSeekVision;
import frc.robot.commands.Tilt.TiltWaitForStop;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.TurretJog;
import frc.robot.commands.Turret.TurretWaitForStop;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
      // The robot's subsystems

      // The driver's controller
      public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
      public final ButtonBox coDriverBox = new ButtonBox(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);

      public final RevDrivetrain m_robotDrive;

      public final RearIntakeSubsystem m_intake;

      public final CellTransportSubsystem m_transport;

      public final RevTurretSubsystem m_turret;

      public final RevTiltSubsystem m_tilt;

      public final RevShooterSubsystem m_shooter;

      public static Preferences prefs;

      public static boolean autoSelected;

      public SetupShuffleboard m_setup;

      public LimeLight m_limelight;

      private Compressor m_compressor;

      private FondyFireTrajectory m_trajectory;

      public AutoFactory m_autoFactory;

      public boolean isMatch = Constants.isMatch;

      public boolean clickUp;

      // Setup gamepad XBox 1
      JoystickButton coDriverA = new JoystickButton(coDriverBox, 1);
      JoystickButton coDriverB = new JoystickButton(coDriverBox, 2);
      JoystickButton coDriverX = new JoystickButton(coDriverBox, 3);
      JoystickButton coDriverY = new JoystickButton(coDriverBox, 4);
      JoystickButton coDriverLeftTrigger = new JoystickButton(coDriverBox, 5);
      JoystickButton coDriverRightTrigger = new JoystickButton(coDriverBox, 6);

      JoystickButton coDriverBack = new JoystickButton(coDriverBox, 7);
      JoystickButton coDriverStart = new JoystickButton(coDriverBox, 8);

      public POVButton coDriverUpButton = new POVButton(coDriverBox, 0);
      public POVButton coDriverRightButton = new POVButton(coDriverBox, 90);
      public POVButton coDriverDownButton = new POVButton(coDriverBox, 180);
      public POVButton coDriverLeftButton = new POVButton(coDriverBox, 270);

      // Setup gamepad XBox 3
      JoystickButton setupA = new JoystickButton(setupGamepad, 1);
      JoystickButton setupB = new JoystickButton(setupGamepad, 2);
      JoystickButton setupX = new JoystickButton(setupGamepad, 3);
      JoystickButton setupY = new JoystickButton(setupGamepad, 4);
      JoystickButton setupLeftTrigger = new JoystickButton(setupGamepad, 5);
      JoystickButton setupRightTrigger = new JoystickButton(setupGamepad, 6);

      JoystickButton setupBack = new JoystickButton(setupGamepad, 7);
      JoystickButton setupStart = new JoystickButton(setupGamepad, 8);

      JoystickButton setupLeftStick = new JoystickButton(setupGamepad, 11);
      JoystickButton setupRightStick = new JoystickButton(setupGamepad, 12);

      public POVButton setupUpButton = new POVButton(setupGamepad, 0);
      public POVButton setupRightButton = new POVButton(setupGamepad, 90);
      public POVButton setupDownButton = new POVButton(setupGamepad, 180);
      public POVButton setupLeftButton = new POVButton(setupGamepad, 270);

      public POVButton driverUpButton = new POVButton(m_driverController, 0);
      public POVButton driverRightButton = new POVButton(m_driverController, 90);
      public POVButton driverDownButton = new POVButton(m_driverController, 180);
      public POVButton driverLeftButton = new POVButton(m_driverController, 270);

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {

            prefs = Preferences.getInstance();
            // Pref.deleteAllPrefs();
            // Pref.deleteUnused();
            Pref.addMissing();
            m_robotDrive = new RevDrivetrain();
            m_transport = new CellTransportSubsystem();
            m_intake = new RearIntakeSubsystem();
            m_shooter = new RevShooterSubsystem();
            m_turret = new RevTurretSubsystem();
            m_tilt = new RevTiltSubsystem();

            m_limelight = new LimeLight();
            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(m_limelight.ledsOffPipeline);
            m_limelight.useVision = false;
            m_compressor = new Compressor();

            m_autoFactory = new AutoFactory(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                        m_compressor, m_intake, 10);

            m_trajectory = new FondyFireTrajectory(m_robotDrive);

            m_tilt.setDefaultCommand(new PositionHoldTilt(m_tilt, m_shooter, m_limelight));

            m_turret.setDefaultCommand(new PositionHoldTurret(m_turret, m_shooter, m_limelight));

            m_shooter.setDefaultCommand(new RunShooter(m_shooter));

            m_setup = new SetupShuffleboard(m_turret, m_tilt, m_robotDrive, m_shooter, m_transport, m_compressor,
                        m_limelight, m_intake, m_trajectory, isMatch);

            m_robotDrive.setDefaultCommand(getArcadeDriveCommand());

            configureButtonBindings();

            LiveWindow.disableAllTelemetry();

            CommandScheduler.getInstance()
                        .onCommandInitialize(command -> System.out.println(command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> System.out.println(command.getName() + " has ended"));
            CommandScheduler.getInstance()
                        .onCommandInterrupt(command -> System.out.println(command.getName() + " was interrupted"));
            CommandScheduler.getInstance().onCommandInitialize(
                        command -> SmartDashboard.putString("CS", command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> SmartDashboard.putString("CE", command.getName() + " has Ended"));
            CommandScheduler.getInstance().onCommandInterrupt(
                        command -> SmartDashboard.putString("CE", command.getName() + "was Interrupted"));

      }

      /**
       * Use this method to define your button->command mappings. Buttons can be
       * created by instantiating a {@link GenericHID} or one of its subclasses
       * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
       * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
       */
      private void configureButtonBindings() {
            if (Robot.isSimulation()) {
                  // for the simulation, silence warnings about missing joysticks
                  DriverStation.getInstance().silenceJoystickConnectionWarning(true);
            }

            /**
             * 
             * Driver Joystick assignments
             * 
             */

            new JoystickButton(m_driverController, 1).whileHeld(new StartIntake(m_intake, m_transport));

            new JoystickButton(m_driverController, 2).whileHeld(
                        new ShootCells(m_shooter, m_tilt, m_turret, m_limelight, m_transport, m_compressor, 100));


            new JoystickButton(m_driverController, 5).whenPressed(new StartShooterWheels(m_shooter, 10));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport));

            new JoystickButton(m_driverController, 4).whenPressed(new PositionTilt(m_tilt, m_tilt.tiltMinAngle))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whenPressed(() -> m_limelight.setPipeline(m_limelight.driverPipeline));

            new JoystickButton(m_driverController, 6).whenPressed(new TiltSeekVision(m_tilt, m_limelight));

            new JoystickButton(m_driverController, 12).whileHeld(() -> m_shooter.setOKShootDriver())
                        .whenReleased(() -> m_shooter.notOKShootDriver());

            // new JoystickButton(m_driverController, 12).whileHeld(

            new JoystickButton(m_driverController, 11).whileHeld(() -> m_shooter.shootAll())
                        .whenReleased(() -> m_shooter.shootOne());

            driverUpButton.whenPressed(() -> m_tilt.aimHigher());

            driverDownButton.whenPressed(() -> m_tilt.aimLower());

            driverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft());// shoot right

            driverRightButton.whenPressed(() -> m_turret.aimFurtherRight());// shoot left

            /**
             * Co driver has miscellaneous functions
             * 
             * 
             * 
             */

            coDriverA.whenPressed(new SetActiveTeleopShootData(4));

            // Shoot for inner goal from 1 meter behind intiation line
            coDriverB.whenPressed(new SetActiveTeleopShootData(1));

            // Trench Shot drive under and just beyond the control panel robot parallel to
            // side wall

            coDriverBack.whenPressed(new SetActiveTeleopShootData(2));

            coDriverBack.whenPressed(new SetActiveTeleopShootData(1));

            // coDriverDownButton.whenPressed(
            // coDriverUpButton.whenPressed(
            // coDriverLeftButton.whileHeld(
            // coDriverRightButton.whenPressed(
            // coDriverStart.whenPressed(

            /**
             * Setup gamepad is used for testing functions
             */

            // setupDownButton.whileHeld(

            setupUpButton.whileHeld(() -> m_transport.runFrontRollerMotor())
                        .whileHeld(() -> m_transport.runRearRollerMotor())
                        .whenReleased(() -> m_transport.stopFrontRollerMotor())
                        .whenReleased(() -> m_transport.stopRearRollerMotor());

            setupLeftButton.whenPressed(new ReleaseOneCell(m_transport));

            setupLeftTrigger.whileHeld(() -> m_transport.pulseLeftBelt(.5, .5, .5))
                        .whenReleased(() -> m_transport.stopLeftBeltMotor());

            setupRightTrigger.whileHeld(() -> m_transport.pulseRightBelt(-.55, .25, .4))
                        .whenReleased(() -> m_transport.stopRightBeltMotor());

            setupBack.whileHeld(new StartIntake(m_intake, m_transport)).whenReleased(new StopIntake(m_intake));

            // setupStart.whileHeld(() -> m_controlPanel.turnWheelMotor())
            // .whenReleased(() -> m_controlPanel.stopWheelMotor());

            setupX.whileHeld(getJogShooterCommand());

            setupY.whileHeld(getJogTiltCommand()).whenReleased(new TiltWaitForStop(m_tilt));

            setupA.whileHeld(getJogTurretCommand()).whenReleased(new TurretWaitForStop(m_turret));

            setupB.whenPressed(new SetUpLimelightForTarget(m_limelight));

            setupLeftStick.whileHeld(getJogLeftBeltCommand());

            setupRightStick.whileHeld(getJogRightBeltCommand());

            // LiveWindow.disableAllTelemetry();

      }

      /**
       * Use this to pass the autonomous command to the main {@link Robot} class.
       *
       * @return the command to run in autonomous
       */

      public Command getAutonomousCommand() {
            return null;

      }

      public Command getArcadeDriveCommand() {
            return new ArcadeDrive(m_robotDrive, () -> -m_driverController.getY(), () -> m_driverController.getTwist());
      }

      public Command getDriveToVisionCommand() {
            return new DriveStraightJoystick(m_robotDrive, () -> -m_driverController.getY());

      }

      public Command getJogTurretCommand() {
            return new TurretJog(m_turret, () -> setupGamepad.getRawAxis(0) / 5, setupGamepad);
      }

      public Command getJogTiltCommand() {
            return new TiltJog(m_tilt, () -> -setupGamepad.getRawAxis(1) / 5, setupGamepad);
      }

      public Command getJogShooterCommand() {
            return new JogShooter(m_shooter, () -> setupGamepad.getRawAxis(4));
      }

      public Command getJogLeftBeltCommand() {
            return new JogLeftBelt(m_transport, () -> setupGamepad.getRawAxis(1));
      }

      public Command getJogRightBeltCommand() {
            return new JogRightBelt(m_transport, () -> setupGamepad.getRawAxis(3));
      }

      public double getThrottle() {
            return (1 - m_driverController.getThrottle()) / 2;
      }

}