// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.TalonSRXWrapper;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */

  final WPI_TalonSRX m_climberMotor = new TalonSRXWrapper(CANConstants.CLIMB_MOTOR);
  final DoubleSolenoid m_climberArm = new DoubleSolenoid(4, 5);

  public ClimberSubsystem() {

    m_climberMotor.configFactoryDefault();

    m_climberMotor.setNeutralMode(NeutralMode.Brake);
    // raiseArm();
    lowerArm();

  }

  public void turnClimberMotor(double speed) {
    m_climberMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getMotorAmps() {
    return m_climberMotor.getStatorCurrent();
  }

  public double getMotorOut() {
    return m_climberMotor.getMotorOutputPercent();
  }


  public void setBrakeOn(boolean on) {
    if (on) {
      m_climberMotor.setNeutralMode(NeutralMode.Brake);
    } else {
      m_climberMotor.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void raiseArm() {
    m_climberArm.set(DoubleSolenoid.Value.kReverse);
  }

  public void lowerArm() {
    m_climberArm.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonSRX(m_climberMotor, 1.5, 7200, true);
  }

  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }
}
