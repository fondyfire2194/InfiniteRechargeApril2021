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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightControlMode.CamMode;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.LimelightControlMode.StreamType;
import frc.robot.commands.AutoCommands.TrenchTwo.ToTrenchTarget;
import frc.robot.commands.CellIntake.IntakeArmLower;
import frc.robot.commands.CellIntake.IntakeArmRaise;
import frc.robot.commands.CellIntake.RunIntakeMotor;
import frc.robot.commands.CellIntake.StartIntake;
import frc.robot.commands.CellIntake.StopIntakeMotor;
import frc.robot.commands.CellTransport.JogLeftBelt;
import frc.robot.commands.CellTransport.JogRightBelt;
import frc.robot.commands.CellTransport.ReleaseLeftArm;
import frc.robot.commands.CellTransport.ReleaseOneCell;
import frc.robot.commands.CellTransport.RunRollers;
import frc.robot.commands.CellTransport.StopBelts;
import frc.robot.commands.CellTransport.StopRollers;
import frc.robot.commands.Climber.ClimberArm;
import frc.robot.commands.Climber.JogClimber;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.ArcadeDriveVelocity;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.Shooter.ChangeShooterSpeed;
import frc.robot.commands.Shooter.ChooseShooterSpeedSource;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.LogShootData;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetShotPosition0;
import frc.robot.commands.Shooter.SetShotPosition2;
import frc.robot.commands.Shooter.ShootCells;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Tilt.TiltJog;
import frc.robot.commands.Tilt.TiltJogVelocity;
import frc.robot.commands.Tilt.TiltWaitForStop;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.TurretJog;
import frc.robot.commands.Turret.TurretJogVelocity;
import frc.robot.commands.Turret.TurretWaitForStop;
import frc.robot.commands.Vision.SetUpLimelightForDriver;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.SetVisionMode;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
      public final XboxController codriverGamepad = new XboxController(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);

      public final RevDrivetrain m_robotDrive;

      public final RearIntakeSubsystem m_intake;

      public final CellTransportSubsystem m_transport;

      public final RevTurretSubsystem m_turret;

      public final RevTiltSubsystem m_tilt;

      public final RevShooterSubsystem m_shooter;

      public final ClimberSubsystem m_climber;

      public static Preferences prefs;

      public static boolean autoSelected;

      public SetupShuffleboard m_setup;

      public LimeLight m_limelight;

      private Compressor m_compressor;

      private FondyFireTrajectory m_trajectory;

      public AutoFactory m_autoFactory;

      public boolean clickUp;

     
      // Co driver gamepad

      // Setup gamepad LOGITECH
      JoystickButton codriverX = new JoystickButton(codriverGamepad, 1);
      JoystickButton codriverA = new JoystickButton(codriverGamepad, 2);
      JoystickButton codriverB = new JoystickButton(codriverGamepad, 3);
      JoystickButton codriverY = new JoystickButton(codriverGamepad, 4);
      JoystickButton codriverLeftTrigger = new JoystickButton(codriverGamepad, 5);
      JoystickButton codriverRightTrigger = new JoystickButton(codriverGamepad, 6);

      JoystickButton codriverBack = new JoystickButton(codriverGamepad, 7);
      JoystickButton codriverStart = new JoystickButton(codriverGamepad, 8);

      JoystickButton codriverLeftStick = new JoystickButton(codriverGamepad, 11);
      JoystickButton codriverRightStick = new JoystickButton(codriverGamepad, 12);

      public POVButton codriverUpButton = new POVButton(codriverGamepad, 0);
      public POVButton codriverRightButton = new POVButton(codriverGamepad, 90);
      public POVButton codriverDownButton = new POVButton(codriverGamepad, 180);
      public POVButton codriverLeftButton = new POVButton(codriverGamepad, 270);

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
            m_climber = new ClimberSubsystem();

            m_limelight = new LimeLight();
            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(m_limelight.ledsOffPipeline);
            m_limelight.useVision = false;
            m_compressor = new Compressor();

            m_autoFactory = new AutoFactory(m_shooter, m_turret, m_tilt, m_transport, m_robotDrive, m_limelight,
                        m_compressor, m_intake);

            m_trajectory = new FondyFireTrajectory(m_robotDrive);

            m_tilt.setDefaultCommand(new PositionHoldTilt(m_tilt, m_shooter, m_limelight));
            // m_tilt.setDefaultCommand(new PositionHoldTiltTest(m_tilt));

            m_turret.setDefaultCommand(new PositionHoldTurret(m_turret, m_shooter, m_limelight));
            // m_turret.setDefaultCommand(new PositionHoldTurretTest(m_turret));

            m_shooter.setDefaultCommand(getJogShooterCommand());
            boolean isMatch = Pref.getPref("IsMatch") == 1.;
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

            new JoystickButton(m_driverController, 2).whileHeld(new StartIntake(m_intake, m_transport));

            new JoystickButton(m_driverController, 1)
                        .whileHeld(new IntakeArmLower(m_intake)).whileHeld(new ShootCells(m_shooter, m_tilt, m_turret,
                                    m_limelight, m_transport, m_robotDrive, m_compressor, 100))
                        .whenReleased(new IntakeArmRaise(m_intake));

            new JoystickButton(m_driverController, 5).whenPressed(new RunShooter(m_shooter))
                        .whenPressed(new RunRollers(m_transport));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport))
                        .whenPressed(new StopRollers(m_transport)).whenPressed(new StopBelts(m_transport))
                        .whenPressed(new SetUpLimelightForNoVision(m_limelight))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whenReleased(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle));

            new JoystickButton(m_driverController, 4).whenPressed(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whenPressed(new SetUpLimelightForNoVision(m_limelight));

            new JoystickButton(m_driverController, 6).whenPressed(
                        new SetUpLimelightForTarget(m_limelight, m_limelight.activeStraightPipeline, true));

            new JoystickButton(m_driverController, 7).whileHeld(new PositionTilt(m_tilt, m_tilt.tiltMinAngle))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whileHeld(new SetUpLimelightForDriver(m_limelight))
                        .whenReleased(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle))
                        .whenReleased(new SetVisionMode(m_limelight))
                        .whenReleased(new SetUpLimelightForNoVision(m_limelight));

            new JoystickButton(m_driverController, 8).whenPressed(
                        new SetUpLimelightForTarget(m_limelight, m_limelight.activeStraightPipeline, true));

            new JoystickButton(m_driverController, 9).whenPressed(new ReleaseLeftArm(m_transport));

            new JoystickButton(m_driverController, 11).whileHeld(() -> m_shooter.shootAll())
                        .whenReleased(() -> m_shooter.shootOne());

            // Hold to shoot all
            new JoystickButton(m_driverController, 12).whileHeld(() -> m_shooter.shootAll())
                        .whenReleased(() -> m_shooter.shootOne());

            driverUpButton.whenPressed(() -> m_tilt.aimHigher());

            driverDownButton.whenPressed(() -> m_tilt.aimLower());

            driverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft());// shoot right

            driverRightButton.whenPressed(() -> m_turret.aimFurtherRight());// shoot left

            /**
             * co driver sets the tilt and turret for shoot positions
             * 
             * 
             */

            // front of power port one meter back
            codriverY.whenPressed(
                        new SetShotPosition0(m_shooter, m_turret, m_tilt, m_transport, m_intake, m_limelight));

            codriverX.whenPressed(() -> m_intake.armSolenoidOff());
        //    codriverA.
            // trench in front of control panel
            codriverB.whenPressed(
                        new SetShotPosition2(m_shooter, m_turret, m_tilt, m_transport, m_intake, m_limelight));

            // control panel

            codriverBack.whileHeld(getRunClimberMotorCommand());

            codriverRightTrigger.whenPressed(new ClimberArm(m_climber, true));

            codriverLeftTrigger.whenPressed(new ClimberArm(m_climber, false));

            codriverUpButton.whenPressed(() -> m_tilt.aimHigher());

            codriverDownButton.whenPressed(() -> m_tilt.aimLower());

            codriverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft());// shoot right

            codriverRightButton.whenPressed(() -> m_turret.aimFurtherRight());// shoot left

            // codriverUpButton.whenPressed(new ChangeShooterSpeed(m_shooter, +1));
            // codriverDownButton.whenPressed(new ChangeShooterSpeed(m_shooter, -1));
            // codriverRightButton.whenPressed(new ChangeShooterSpeed(m_shooter, +2));
            // codriverLeftButton.whenPressed(new ChangeShooterSpeed(m_shooter, -2));
            /**
             * Setup gamepad is used for testing functions
             */

            setupDownButton.whenPressed(new ParallelCommandGroup((new PositionTiltToVision(m_tilt, m_limelight, 24)),
                        new PositionTurretToVision(m_turret, m_limelight, -32)));

            setupUpButton.whileHeld(() -> m_transport.runFrontRollerMotor(.5))
                        .whileHeld(() -> m_transport.runRearRollerMotor(.5))
                        .whenReleased(() -> m_transport.stopFrontRollerMotor())
                        .whenReleased(() -> m_transport.stopRearRollerMotor());

            setupLeftButton.whenPressed(new ReleaseOneCell(m_transport));

            // setupLeftTrigger.

            // setupRightTrigger.

            // setupBack.

            setupX.whileHeld(getJogTiltVelocityCommand(setupGamepad))
                        .whileHeld(getJogTurretVelocityCommand(setupGamepad)).whenReleased(new TiltWaitForStop(m_tilt))
                        .whenReleased(new TurretWaitForStop(m_turret));

            setupY.whileHeld(getJogTiltCommand(setupGamepad)).whileHeld(getJogTurretCommand(setupGamepad))
                        .whenReleased(new TiltWaitForStop(m_tilt)).whenReleased(new TurretWaitForStop(m_turret));

            // setupB.whenPressed(

            setupA.whileHeld(new RunIntakeMotor(m_intake, .75)).whenReleased(new StopIntakeMotor(m_intake));

            // setupLeftStick.

            // setupRightStick.

            LiveWindow.disableAllTelemetry();

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

      public Command getArcadeDriveVelocityCommand() {
            return new ArcadeDriveVelocity(m_robotDrive, () -> -m_driverController.getY(),
                        () -> m_driverController.getTwist());
      }

      public Command getDriveStraightCommand() {
            return new DriveStraightJoystick(m_robotDrive, () -> -m_driverController.getY());

      }

      public Command getJogTurretCommand(XboxController gamepad) {
            return new TurretJog(m_turret, () -> gamepad.getRawAxis(0) / 5, gamepad);
      }

      public Command getJogTiltCommand(XboxController gamepad) {
            return new TiltJog(m_tilt, () -> -gamepad.getRawAxis(1) / 5, gamepad);
      }

      /**
       * Use to tune velocity loop for use inside vision position loops. Goal is to
       * correct 5 degrees of vision error in 1 second.
       * 
       * One tilt motor rev is .25 degrees. 11000rpm = 180 rps = 45 dps max
       * 
       * 5 degrees vision = 20 motor revs per second.,approx 10% full speed
       * 
       * 
       * 
       * One turret motor rev is 1.41 degrees. 11000 rpm = 180 rps = 250 dps
       * 
       * So 5 degrees in one second 5 approx 1/150 or 7% full speed
       * 
       * 
       * 
       * 
       * @param gamepad
       * @return
       */

      public Command getJogTurretVelocityCommand(XboxController gamepad) {
            return new TurretJogVelocity(m_turret, () -> -gamepad.getRawAxis(0), gamepad);
      }

      public Command getJogTiltVelocityCommand(XboxController gamepad) {
            return new TiltJogVelocity(m_tilt, () -> -gamepad.getRawAxis(1), gamepad);
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

      public Command getRunClimberMotorCommand() {
            return new JogClimber(m_climber, () -> codriverGamepad.getRawAxis(3));
      }

      public double getThrottle() {
            return (1 - m_driverController.getThrottle()) / 2;
      }

}