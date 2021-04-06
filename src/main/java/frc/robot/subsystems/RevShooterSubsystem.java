package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.sim.ShooterSubsystem;
import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class RevShooterSubsystem extends SubsystemBase implements ShooterSubsystem {
    private final SimableCANSparkMax mLeftMotor; // NOPMD
    private final SimableCANSparkMax mRightMotor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mSimulator;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    public static double kGearing = 1;
    public static double kInertia = 0.008;

    public RevShooterSubsystem() {
        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mRightMotor.follow(mLeftMotor);

        mEncoder = mLeftMotor.getEncoder(EncoderType.kQuadrature, 8192);
        mPidController = mLeftMotor.getPIDController();

        mPidController.setP(0.0005);
        mPidController.setFF(1.0 / 4700);

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }
    }

    @Override
    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    @Override
    public void spinAtRpm(double rpm) {
        mPidController.setReference(rpm, ControlType.kVelocity);
    }

    @Override
    public double getRPM() {
        return mEncoder.getVelocity();
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
    }

    @Override
    public void stop() {
        mLeftMotor.set(0);
    }
}
