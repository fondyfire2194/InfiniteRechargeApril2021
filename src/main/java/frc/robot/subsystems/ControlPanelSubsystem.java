/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.TalonSRXWrapper;

public class ControlPanelSubsystem extends SubsystemBase {
   /**
    * Creates a new ControlPanel.
    */
   /**
    * Change the I2C port below to match the connection of your color sensor
    */
   private final I2C.Port i2cPort = I2C.Port.kOnboard;
   /**
    * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
    * The device will be automatically initialized with default parameters.
    */
   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

   /**
    * A Rev Color Match object is used to register and detect known colors. This
    * can be calibrated ahead of time or during operation.
    * 
    * This object uses a simple euclidian distance to estimate the closest match
    * with given confidence range.
    */
   private final ColorMatch m_colorMatcher = new ColorMatch();
   private Color detectedColor;

   public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

   String colorString;
   public int colorNumber = 0;
   public int lastColorNumber;
   public int colorNumberFiltered;
   private int loopCount;

   private int filterNumber = 3;
   private int redCount;
   private int yellowCount;
   private int blueCount;
   private int greenCount;

   public int gameColorNumber;

   public String[] seenColor = { "grey", "blue", "green", "red", "yellow" };

   public boolean lookForColor;

   private WPI_TalonSRX m_controlPanelMotor = new TalonSRXWrapper(CANConstants.CP_TURN_MOTOR);
   private final DoubleSolenoid m_colorWheelArm = new DoubleSolenoid(0, 1);
   private int simColorCount;
   SimpleWidget colorWidget;
   NetworkTableEntry colorWidgetEntry;
   private boolean doneOnce;
   private int colorNumberLast;

   SimpleWidget gameColorWidget;
   NetworkTableEntry gameColorWidgetEntry;
   private boolean gameDoneOnce;
   private int gameColorNumberLast;
   public int revsDone;

   public ControlPanelSubsystem() {

      m_controlPanelMotor.configFactoryDefault();
      m_controlPanelMotor.setNeutralMode(NeutralMode.Brake);
      raiseArm();
      m_colorMatcher.addColorMatch(kBlueTarget);
      m_colorMatcher.addColorMatch(kGreenTarget);
      m_colorMatcher.addColorMatch(kRedTarget);
      m_colorMatcher.addColorMatch(kYellowTarget);

      colorWidget = Shuffleboard.getTab("SetupClimber_CP").add("SensorColor", false).withWidget("Boolean Box")
            .withPosition(8, 0).withProperties(Map.of("colorWhenFalse", "maroon"));
      colorWidgetEntry = colorWidget.getEntry();
      colorWidgetEntry.getBoolean(false);

      gameColorWidget = Shuffleboard.getTab("SetupClimber_CP").add("Game Color", false).withWidget("Boolean Box")
            .withPosition(8, 1).withProperties(Map.of("colorWhenFalse", "maroon"));
      gameColorWidgetEntry = gameColorWidget.getEntry();
      gameColorWidgetEntry.getBoolean(false);
   }

   public void turnWheelMotor(double speed) {
      m_controlPanelMotor.set(ControlMode.PercentOutput, speed);

   }

   public double getMotorSet() {
      return m_controlPanelMotor.get();
   }

   public double getMotorAmps() {
      if (Robot.isReal()) {
         return m_controlPanelMotor.getStatorCurrent();
      } else
         return 0;
   }

   public void setBrakeOn(boolean on) {
      if (on) {
         m_controlPanelMotor.setNeutralMode(NeutralMode.Brake);
      } else {
         m_controlPanelMotor.setNeutralMode(NeutralMode.Coast);
      }
   }

   public void raiseArm() {
      m_colorWheelArm.set(DoubleSolenoid.Value.kReverse);
   }

   public void lowerArm() {
      m_colorWheelArm.set(DoubleSolenoid.Value.kForward);
   }

   public boolean getArmLowered() {
      return m_colorWheelArm.get() == DoubleSolenoid.Value.kForward;
   }

   public boolean getArmRaised() {
      return m_colorWheelArm.get() == DoubleSolenoid.Value.kReverse;
   }

   public double getSensorDistance() {
      return m_colorSensor.getProximity();
   }

   public double getSensorIR() {
      return m_colorSensor.getIR();
   }

   public void setLookForColor(boolean on) {
      lookForColor = on;
   }

   public void simulationInit() {
      PhysicsSim.getInstance().addTalonSRX(m_controlPanelMotor, 1.5, 7200, true);
   }

   public void simulationPeriodic() {
      if (lookForColor) {
         m_controlPanelMotor.set(.25);
      } else {
         m_controlPanelMotor.set(0);
      }
      PhysicsSim.getInstance().run();
      if (lookForColor && getMotorSet() != 0) {
         simColorCount++;
         if (simColorCount > 200) {
            colorNumber++;
            simColorCount = 0;
         }
         if (colorNumber > 4)
            colorNumber = 0;
      }
   }

   @Override
   public void periodic() {

      loopCount++;
      if (loopCount > 5 && lookForColor) {
         // This method will be called once per scheduler run
         if (!Robot.isSimulation()) {
            detectedColor = m_colorSensor.getColor();
            // "grey", "blue", "green", "red", "yellow"
            ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

            if (match.color == kBlueTarget) {
               colorNumber = 1;
            } else if (match.color == kGreenTarget) {
               colorNumber = 2;
            } else if (match.color == kRedTarget) {
               colorNumber = 3;
            } else if (match.color == kYellowTarget) {
               colorNumber = 4;
            } else
               colorNumber = 0;

         }

         if (colorNumber != lastColorNumber) {

         }
         String temp = seenColor[colorNumber];

         int ourTargetColor = gameColorNumber + 2;
         if (ourTargetColor > 4)
            ourTargetColor -= 4;

      }
      filterColors();

      if (colorNumberFiltered != colorNumberLast)

      {
         colorNumberLast = colorNumberFiltered;
         doneOnce = false;
      }
      if (!doneOnce) {
         colorWidget.withProperties(Map.of("colorWhenTrue", seenColor[colorNumber]));
         colorWidgetEntry.setBoolean(true);
         doneOnce = true;
      }

      getGameData();

      if (gameColorNumber != gameColorNumberLast)

      {
         gameColorNumberLast = gameColorNumber;
         gameDoneOnce = false;
      }
      if (!gameDoneOnce) {
         gameColorWidget.withProperties(Map.of("colorWhenTrue", seenColor[gameColorNumber]));
         gameColorWidgetEntry.setBoolean(true);
         gameDoneOnce = true;
      }

   }

   public void filterColors() {
      if (colorNumber == 1) {
         blueCount++;
         if (blueCount >= filterNumber) {
            colorNumberFiltered = 1;
            redCount = 0;
            yellowCount = 0;
            greenCount = 0;
         }
      }
      if (colorNumber == 2) {
         greenCount++;
         if (greenCount >= filterNumber) {
            colorNumberFiltered = 2;
            redCount = 0;
            blueCount = 0;
            yellowCount = 0;
         }
      }
      if (colorNumber == 3) {
         redCount++;
         if (redCount >= filterNumber) {
            colorNumberFiltered = 3;
            yellowCount = 0;
            blueCount = 0;
            greenCount = 0;
         }
      }

      if (colorNumber == 4) {
         yellowCount++;

         if (yellowCount >= filterNumber) {
            colorNumberFiltered = 4;
            redCount = 0;
            blueCount = 0;
            greenCount = 0;
         }
      }

   }

   public int getGameData() {
      String gameData;

      gameData = DriverStation.getInstance().getGameSpecificMessage();
      if (gameData.length() > 0) {

         switch (gameData.charAt(0)) {
         case 'B':
            // Blue case code
            gameColorNumber = 1;

            break;
         case 'G':
            // Green case code
            gameColorNumber = 2;
            break;
         case 'R':
            // Red case code
            gameColorNumber = 3;
            break;
         case 'Y':
            // Yellow case code
            gameColorNumber = 4;
            break;
         default:
            // This is corrupt data
            gameColorNumber = 0;
            break;
         }
      } else {
         // Code for no data received yet

      }
      return gameColorNumber;
   }

}
