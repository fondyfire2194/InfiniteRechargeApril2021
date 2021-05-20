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
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
import frc.robot.commands.CellTransport.CellBeltPulseSelect;
import frc.robot.commands.Climber.ClimberArm;
import frc.robot.commands.Climber.TurnClimberMotor;
import frc.robot.commands.ControlPanel.ControlPanelArm;
import frc.robot.commands.ControlPanel.PositionNumberRevs;
import frc.robot.commands.ControlPanel.PositionToColor;
import frc.robot.commands.ControlPanel.TurnControlPanel;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.Shooter.ChangeShooterSpeed;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.RunShooterWheels;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StartShooterWheels;
import frc.robot.commands.Shooter.StopShooterWheels;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.TiltJog;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Tilt.TiltWaitForStop;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.TurretJog;
import frc.robot.commands.Turret.TurretJogVelocity;
import frc.robot.commands.Turret.TurretWaitForStop;
import frc.robot.commands.Vision.LimelightCamMode;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
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
      public final XboxController coDriverGamepad = new XboxController(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);

      public final RevDrivetrain m_robotDrive;

      public final ClimberSubsystem m_climber;

      public final ControlPanelSubsystem m_controlPanel;

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

      public boolean isMatch = false;

      public boolean clickUp;

      // Setup gamepad XBox 1
      JoystickButton coDriverA = new JoystickButton(coDriverGamepad, 1);
      JoystickButton coDriverB = new JoystickButton(coDriverGamepad, 2);
      JoystickButton coDriverX = new JoystickButton(coDriverGamepad, 3);
      JoystickButton coDriverY = new JoystickButton(coDriverGamepad, 4);
      JoystickButton coDriverLeftTrigger = new JoystickButton(coDriverGamepad, 5);
      JoystickButton coDriverRightTrigger = new JoystickButton(coDriverGamepad, 6);

      JoystickButton coDriverBack = new JoystickButton(coDriverGamepad, 7);
      JoystickButton coDriverStart = new JoystickButton(coDriverGamepad, 8);

      public POVButton coDriverUpButton = new POVButton(coDriverGamepad, 0);
      public POVButton coDriverRightButton = new POVButton(coDriverGamepad, 90);
      public POVButton coDriverDownButton = new POVButton(coDriverGamepad, 180);
      public POVButton coDriverLeftButton = new POVButton(coDriverGamepad, 270);

      // Setup gamepad XBox 3
      JoystickButton setupA = new JoystickButton(setupGamepad, 1);
      JoystickButton setupB = new JoystickButton(setupGamepad, 2);
      JoystickButton setupX = new JoystickButton(setupGamepad, 3);
      JoystickButton setupY = new JoystickButton(setupGamepad, 4);
      JoystickButton setupLeftTrigger = new JoystickButton(setupGamepad, 5);
      JoystickButton setupRightTrigger = new JoystickButton(setupGamepad, 6);

      JoystickButton setupBack = new JoystickButton(setupGamepad, 7);
      JoystickButton setupStart = new JoystickButton(setupGamepad, 8);

      public POVButton setupUpButton = new POVButton(setupGamepad, 0);
      public POVButton setupRightButton = new POVButton(setupGamepad, 90);
      public POVButton setupDownButton = new POVButton(setupGamepad, 180);
      public POVButton setupLeftButton = new POVButton(setupGamepad, 270);

      public POVButton driverUpButton = new POVButton(m_driverController, 0);
      public POVButton driverRightButton = new POVButton(m_driverController, 90);
      public POVButton driverDownButton = new POVButton(m_driverController, 180);
      public POVButton driverLeftButton = new POVButton(m_driverController, 270);

      // AutoCommands ac;// = new AutoCommands(m_robotDrive);
      public int shootPosition;

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {

            prefs = Preferences.getInstance();
            // Pref.deleteAllPrefs();
            Pref.deleteUnused();
            Pref.addMissing();
            m_robotDrive = new RevDrivetrain();

            m_climber = new ClimberSubsystem();
            m_transport = new CellTransportSubsystem();
            m_controlPanel = new ControlPanelSubsystem();
            m_intake = new RearIntakeSubsystem();
            m_shooter = new RevShooterSubsystem();
            m_turret = new RevTurretSubsystem();
            m_tilt = new RevTiltSubsystem();

            m_limelight = new LimeLight();
            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(1);

            m_compressor = new Compressor();

            m_autoFactory = new AutoFactory(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                        m_compressor, m_intake, 0);

            m_trajectory = new FondyFireTrajectory(m_robotDrive);

            m_tilt.setDefaultCommand(new PositionHoldTilt(m_tilt, m_limelight));

            m_turret.setDefaultCommand(new PositionHoldTurret(m_turret, m_limelight));

            // m_shooter.setDefaultCommand(getJogShooterCommand());

            m_setup = new SetupShuffleboard(m_turret, m_tilt, m_robotDrive, m_shooter, m_transport, m_compressor,
                        m_limelight, m_controlPanel, m_intake, m_trajectory, m_climber, isMatch);

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

            new JoystickButton(m_driverController, 1).whileHeld(new StartIntake(m_intake, m_limelight));

            new JoystickButton(m_driverController, 2)
                        .whileHeld(new ShootCells(m_shooter, m_transport, m_compressor, 100));

            new JoystickButton(m_driverController, 5).whenPressed(new RunShooterWheels(m_shooter));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShooterWheels(m_shooter));

            new JoystickButton(m_driverController, 6).whenPressed(new ChangeShooterSpeed(m_shooter, +.1));

            new JoystickButton(m_driverController, 4).whenPressed(new ChangeShooterSpeed(m_shooter, -.1));

            new JoystickButton(m_driverController, 7).whenPressed(() -> m_tilt.aimCenter())
                        .whenPressed(() -> m_turret.aimCenter());

            new JoystickButton(m_driverController, 8).whenPressed(new LimelightCamMode(m_limelight, CamMode.kdriver));

            new JoystickButton(m_driverController, 9).whenPressed(new LimelightCamMode(m_limelight, CamMode.kvision));

            driverUpButton.whenPressed(() -> m_tilt.aimHigher(.05));
            driverDownButton.whenPressed(() -> m_tilt.aimLower(.05));

            driverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft(.05));
            driverRightButton.whenPressed(() -> m_turret.aimFurtherRight(.05));
            /**
             * Co driver has miscellaneous functions
             * 
             * 
             * 
             */
            coDriverA.whenPressed(new ControlPanelArm(m_controlPanel, true));
            coDriverB.whenPressed(new ControlPanelArm(m_controlPanel, false));
   
            coDriverX.whileHeld(new TurnControlPanel(m_controlPanel,.25));

            coDriverX.whenPressed(new PositionToColor(m_controlPanel,.25));
            coDriverBack.whenPressed(new PositionNumberRevs(m_controlPanel, 3, .25));




            /**
             * 
             * Setup gamepad is used for testing functions
             * 
             * 
             */
            setupDownButton

                        .whenPressed(() -> m_transport.runFrontRollerMotor(-.5))
                        .whenPressed(() -> m_transport.runRearRollerMotor(.5))
                        .whenReleased(() -> m_transport.runFrontRollerMotor(0))
                        .whenReleased(() -> m_transport.runRearRollerMotor(0));

            setupUpButton

                        .whenPressed(() -> m_transport.runLeftBeltMotor(.5))
                        .whenPressed(() -> m_transport.runRightBeltMotor(.5))

                        .whenReleased(() -> m_transport.runLeftBeltMotor(0))
                        .whenReleased(() -> m_transport.runLeftBeltMotor(0));

            setupLeftButton.whileHeld(new CellBeltPulseSelect(m_transport, true, 1, .25, 1, -.25));

            setupRightButton.whileHeld(new CellBeltPulseSelect(m_transport, false, 1, .25, 1, -.25));

            setupLeftTrigger.whileHeld(new ClimberArm(m_climber, true)).whileHeld(new TurnClimberMotor(m_climber, .25));

            setupRightTrigger.whileHeld(new ClimberArm(m_climber, false))
                        .whileHeld(new TurnClimberMotor(m_climber, -.25));

            setupBack.whileHeld(new StartIntake(m_intake, m_limelight)).whenReleased(new StopIntake(m_intake));

            setupStart.whenPressed(new TiltMoveToReverseLimit(m_tilt));

            setupX.whileHeld(getJogShooterCommand());

            setupY.whileHeld(getJogTiltCommand()).whenReleased(new TiltWaitForStop(m_tilt));

            setupA.whileHeld(getJogTurretCommand()).whenReleased(new TurretWaitForStop(m_turret));

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

      public Command getJogTurretVelocityCommand() {
            return new TurretJogVelocity(m_turret, () -> setupGamepad.getRawAxis(0) / 2);
      }

      public Command getJogTurretCommand() {
            return new TurretJog(m_turret, () -> setupGamepad.getRawAxis(0) / 5, setupGamepad);
      }

      public Command getJogTiltCommand() {
            return new TiltJog(m_tilt, () -> setupGamepad.getRawAxis(1) / 5, setupGamepad);
      }

      public Command getJogShooterCommand() {
            return new JogShooter(m_shooter, () -> -setupGamepad.getRawAxis(5) / 2);

      }

}