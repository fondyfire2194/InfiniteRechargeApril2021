/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.TalonSRXWrapper;

public class RearIntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new RearIntake.
   */
  private final WPI_TalonSRX m_intakeMotor = new TalonSRXWrapper(CANConstants.REAR_MOTOR);
  public final DoubleSolenoid m_intakeArm = new DoubleSolenoid(2, 3);
  public boolean intakeMotorConnected;

  public RearIntakeSubsystem() {

    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    raiseArm();
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonSRX(m_intakeMotor, 1.5, 7200, true);
  }

  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public boolean checkCAN() {
    return intakeMotorConnected = m_intakeMotor.getFirmwareVersion() != -1;

  }

  public void runIntakeMotor(double speed) {
    m_intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntakeMotor() {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getMotor() {
    return m_intakeMotor.getMotorOutputPercent();
  }

  public double getMotorAmps() {
    return m_intakeMotor.getStatorCurrent();
  }

  public void setBrakeOn(boolean on) {
    if (on) {
      m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_intakeMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void raiseArm() {
    m_intakeArm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lowerArm() {
    m_intakeArm.set(DoubleSolenoid.Value.kForward);
  }

  public boolean getArmLowered() {
    return m_intakeArm.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getArmRaised() {
    return m_intakeArm.get() == DoubleSolenoid.Value.kReverse;
  }

  public void armSolenoidOff() {
    m_intakeArm.set(DoubleSolenoid.Value.kOff);
  }

  public boolean getArmOff() {
    return m_intakeArm.get() == DoubleSolenoid.Value.kOff;
  }

}
