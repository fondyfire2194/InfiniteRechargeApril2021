/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.JogTilt;
import frc.robot.commands.PositionTilt;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

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
      public final XboxController gamepad = new XboxController(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);
      public final XboxController shootBox = new XboxController(OIConstants.kShootBoxControllerPort);

      public final RevDrivetrain m_driveTrain;

      public final ClimberSubsystem m_climber;

      public final ControlPanelSubsystem m_controlPanel;

      public final RearIntakeSubsystem m_intake;

      public final CellTransportSubsystem m_transport;

      public final RevTurretSubsystem m_turret;

      public final RevTiltSubsystem m_revTilt;
      //
      public static Preferences prefs;

      public static boolean autoSelected;

      // AutoCommands ac;// = new AutoCommands(m_robotDrive);
      public int shootPosition;

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {
            prefs = Preferences.getInstance();
            // Pref.deleteAllPrefs();
            // Pref.deleteUnused();
            // Pref.addMissing();
            m_driveTrain = new RevDrivetrain();
            m_climber = new ClimberSubsystem();
            m_transport = new CellTransportSubsystem();
            m_controlPanel = new ControlPanelSubsystem();
            m_intake = new RearIntakeSubsystem();

            m_turret = new RevTurretSubsystem();

            m_revTilt = new RevTiltSubsystem();
            SmartDashboard.putData(m_revTilt);

            m_revTilt.setDefaultCommand(new PositionTilt(m_revTilt).withName("Def"));

            configureButtonBindings();
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
            // Driver Joystick

            // new JoystickButton(m_driverController, 1).whileHeld(new
            // StartRearIntake(m_rearIntake));

            // co driver gamepad
            new JoystickButton(m_driverController, 1)

                        .whenPressed(() -> m_climber.turnClimberMotor(.5))
                        .whenPressed(() -> m_controlPanel.turnWheelMotor(.5))

                        .whileHeld(new JogTilt(m_revTilt, .4).withName("Jogging"))
                        .whenReleased(() -> m_climber.turnClimberMotor(0))
                        .whenReleased(() -> m_controlPanel.turnWheelMotor(0)).whenReleased(() -> m_revTilt.stop());

            new JoystickButton(m_driverController, 2)

                        .whenPressed(() -> m_intake.runIntakeMotor(.5)).whenReleased(() -> m_intake.runIntakeMotor(0));

            new JoystickButton(m_driverController, 3)

                        .whenPressed(() -> m_transport.runFrontRollerMotor(.5))
                        .whenPressed(() -> m_transport.runRearRollerMotor(.5))
                        .whenPressed(() -> m_transport.runLeftBeltMotor(.5))
                        .whenPressed(() -> m_transport.runRightBeltMotor(.5))
                        .whenPressed(() -> m_intake.runIntakeMotor(.5))
                        .whenReleased(() -> m_transport.runFrontRollerMotor(0))
                        .whenReleased(() -> m_transport.runRearRollerMotor(0))
                        .whenReleased(() -> m_transport.runLeftBeltMotor(0))
                        .whenPressed(() -> m_intake.runIntakeMotor(0))
                        .whenReleased(() -> m_transport.runRightBeltMotor(0));

            new JoystickButton(m_driverController, 8)

                        .whileHeld(() -> m_revTilt.moveManually(.4), m_revTilt);

            new JoystickButton(m_driverController, 7)

                        .whenPressed(new PositionTilt(m_revTilt, .1).withName("SM"));

            // CommandScheduler.getInstance()
            // .onCommandInitialize(command -> System.out.println(command.getName() + " is
            // starting"));
            // CommandScheduler.getInstance()
            // .onCommandFinish(command -> System.out.println(command.getName() + " has
            // ended"));
            // CommandScheduler.getInstance()
            // .onCommandInterrupt(command -> System.out.println(command.getName() + " was
            // interrupted"));
            // CommandScheduler.getInstance().onCommandInitialize(
            // command -> SmartDashboard.putString("CS", command.getName() + " is
            // starting"));
            // CommandScheduler.getInstance()
            // .onCommandFinish(command -> SmartDashboard.putString("CE", command.getName()
            // + " has Ended"));
            // CommandScheduler.getInstance().onCommandInterrupt(
            // command -> SmartDashboard.putString("CE", command.getName() + "was
            // Interrupted"));

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

}
