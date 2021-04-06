/*
 * Create a wrapper around the CANSparkMax class to support simulation
 */

package frc.robot.simulation;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotController;

public class TalonSRXWrapper extends WPI_TalonSRX{
    private SimDouble m_simSpeed;
    private SimDevice m_talonSRX;

    public TalonSRXWrapper(int deviceID) {
        super(deviceID);

        m_talonSRX = SimDevice.create("TalonSRX",deviceID);
        if (m_talonSRX != null){
            m_simSpeed = m_talonSRX.createDouble("speed", false, 0.0);
            
        }
    }

    @Override
    public double get(){
        if (m_talonSRX != null){
            return m_simSpeed.get();
        }
        return super.get();
    }
   

    @Override
    public void set(double speed){
        if (m_talonSRX != null){
            m_simSpeed.set(speed);
        }else{
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (m_talonSRX != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}