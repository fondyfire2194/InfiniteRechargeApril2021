package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterSubsystem extends Subsystem, AutoCloseable
{

    final class FlywheelSimConstants
    {
        public static  DCMotor kGearbox = DCMotor.getVex775Pro(2);
        public static  double kGearing = 4;
        public static  double kInertia = 0.008;

        public static FlywheelSim createSim()
        {
            return new FlywheelSim(FlywheelSimConstants.kGearbox, FlywheelSimConstants.kGearing, FlywheelSimConstants.kInertia);
        }

        private FlywheelSimConstants()
        {

        }
    }

    void spinAtRpm(double rpm);

    double getRPM();

    void stop();
}
