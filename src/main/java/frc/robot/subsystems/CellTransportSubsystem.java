/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.TalonSRXWrapper;

public class CellTransportSubsystem extends SubsystemBase {
  /**
   * Creates a new CellTransport.
   */
  private final WPI_TalonSRX m_leftBeltMotor;
  private final WPI_TalonSRX m_rightBeltMotor;
  private final WPI_TalonSRX m_frontRollerMotor;
  private final WPI_TalonSRX m_rearRollerMotor;

  public List<BaseTalon> transportTalons;

  private double beltPulseStartTime;
  public boolean leftBeltMotorConnected;
  public boolean rightBeltMotorConnected;
  public boolean frontRollerMotorConnected;
  public boolean rearRollerMotorConnected;
  private int loopCtr;

  public CellTransportSubsystem() {
    m_leftBeltMotor = new TalonSRXWrapper(CANConstants.LEFT_BELT_MOTOR);
    m_rightBeltMotor = new TalonSRXWrapper(CANConstants.RIGHT_BELT_MOTOR);
    m_frontRollerMotor = new TalonSRXWrapper(CANConstants.FRONT_ROLLER);
    m_rearRollerMotor = new TalonSRXWrapper(CANConstants.REAR_ROLLER);

    if (Robot.isReal()) {
      m_leftBeltMotor.configFactoryDefault();
      m_rightBeltMotor.configFactoryDefault();
      m_frontRollerMotor.configFactoryDefault();
      m_rearRollerMotor.configFactoryDefault();
      m_leftBeltMotor.setNeutralMode(NeutralMode.Brake);
      m_rightBeltMotor.setNeutralMode(NeutralMode.Brake);
      m_frontRollerMotor.setNeutralMode(NeutralMode.Brake);
      m_rearRollerMotor.setNeutralMode(NeutralMode.Brake);

      setFrontRollerBrakeOn(true);
      setRearRollerBrakeOn(true);
      setBeltBrakeOn(true);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    loopCtr++;

  }

  public boolean checkCAN() {

    leftBeltMotorConnected = m_leftBeltMotor.getFirmwareVersion() != -1;
    rightBeltMotorConnected = m_rightBeltMotor.getFirmwareVersion() != -1;
    frontRollerMotorConnected = m_frontRollerMotor.getFirmwareVersion() != -1;
    rearRollerMotorConnected = m_rearRollerMotor.getFirmwareVersion() != -1;

    return leftBeltMotorConnected && rightBeltMotorConnected && frontRollerMotorConnected && rearRollerMotorConnected;

  }

  public void runLeftBeltMotor(double speed) {
    m_leftBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getLeftBelt() {
    return m_leftBeltMotor.getMotorOutputPercent();
  }

  public void runRightBeltMotor(double speed) {
    m_rightBeltMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getRightBelt() {
    return m_rightBeltMotor.getMotorOutputPercent();
  }

  public double getLeftBeltMotorAmps() {
    return m_leftBeltMotor.getStatorCurrent();
  }

  public double getRightBeltMotorAmps() {
    if (Robot.isReal()) {
      return m_rightBeltMotor.getStatorCurrent();
    } else {
      return 0;
    }
  }

  public void pulseBelt(double speed, double onTime, double offTime) {
    if (beltPulseStartTime == 0)
      beltPulseStartTime = Timer.getFPGATimestamp();

    if (Timer.getFPGATimestamp() < beltPulseStartTime + onTime) {
      runLeftBeltMotor(-speed);
    }

    if (Timer.getFPGATimestamp() > beltPulseStartTime + onTime
        && Timer.getFPGATimestamp() < beltPulseStartTime + onTime + offTime) {
      runLeftBeltMotor(0);
    }

    if (Timer.getFPGATimestamp() > beltPulseStartTime + onTime + offTime) {
      beltPulseStartTime = 0;
    }

  }

  public void setBeltBrakeOn(boolean on) {
    if (on) {
      m_leftBeltMotor.setNeutralMode(NeutralMode.Brake);
      m_rightBeltMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_leftBeltMotor.setNeutralMode(NeutralMode.Coast);
      m_rightBeltMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void runFrontRollerMotor(double speed) {
    m_frontRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public double getFrontRoller() {
    return m_frontRollerMotor.getMotorOutputPercent();
  }

  public void setFrontRollerBrakeOn(boolean on) {
    if (on) {
      m_frontRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_frontRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void runRearRollerMotor(double speed) {
    m_rearRollerMotor.set(ControlMode.PercentOutput, -speed);
  }

  public double getRearRoller() {
    return m_rearRollerMotor.getMotorOutputPercent();
  }

  public void setRearRollerBrakeOn(boolean on) {
    if (on) {
      m_rearRollerMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_rearRollerMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getFrontRollerMotorAmps() {
    return m_rearRollerMotor.getStatorCurrent();
  }

  public double getRearRollerMotorAmps() {
    return m_frontRollerMotor.getStatorCurrent();
  }

  public void simulationInit() {

    PhysicsSim.getInstance().addTalonSRX(m_frontRollerMotor, 1.5, 7200, true);
    PhysicsSim.getInstance().addTalonSRX(m_leftBeltMotor, 1.5, 7200, true);
    PhysicsSim.getInstance().addTalonSRX(m_rightBeltMotor, 1.5, 7200, true);
    PhysicsSim.getInstance().addTalonSRX(m_rearRollerMotor, 1.5, 7200, true);

  }

  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
