package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.FlywheelSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.ShooterSubsystem;

public class RevShooterSubsystem extends SubsystemBase implements ShooterSubsystem {
    public final SimableCANSparkMax mLeftMotor; // NOPMD
    // private final SimableCANSparkMax mRightMotor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mSimulator;
    public double requiredSpeedLast;
    public double requiredSpeed;
    public double shootTime;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    public static double kGearing = 1;
    public static double kInertia = 0.008;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private SimpleWidget shootColorWidget;
    private NetworkTableEntry shootColorWidgetEntry;
    private boolean doneOnce;

    public String[] shootColor = { "red", "yellow", "green" };
    public int shootColorNumber;
    private int shootColorNumberLast = 1;

    public double calculatedCameraDistance;

    public RevShooterSubsystem() {
        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        // mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR,
        // CANSparkMaxLowLevel.MotorType.kBrushless);

        // mRightMotor.follow(mLeftMotor);

        mEncoder = mLeftMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        // mRightMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(5.);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        // mRightMotor.setIdleMode(IdleMode.kBrake) ;
        // mPidController.setP(0.001);
        // mPidController.setFF(1.0 / 4700);
        setGains();
        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }

        shootColorWidget = Shuffleboard.getTab("Competition").add("ShootColor", false).withWidget("Boolean Box")
                .withPosition(0, 0).withSize(2, 2).withProperties(Map.of("colorWhenFalse", "black"));
        shootColorWidgetEntry = shootColorWidget.getEntry();
        shootColorWidgetEntry.getBoolean(false);

    }

    public void calibratePID(final double p, final double i, final double d, final double f) {
        mPidController.setIAccum(0);
        mPidController.setP(p);
        mPidController.setI(i);
        mPidController.setD(d);
        mPidController.setFF(f);
        mPidController.setIZone(1000);
    }

    @Override
    public void close() {
        mLeftMotor.close();
        // mRightMotor.close();
    }

    @Override
    public void spinAtRpm(double rpm) {
        requiredSpeed = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity);
    }

    public void moveManually(double speed) {
        mLeftMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        shootColorNumber = (int) SmartDashboard.getNumber("ShootColor", 0);
        if (shootColorNumber > 2)
            shootColorNumber = 2;
        if (shootColorNumber != shootColorNumberLast) {
            if (shootColorNumber == 0) {
                shootColorWidgetEntry.setBoolean(false);
                shootColorNumberLast = shootColorNumber;
            } else
                doneOnce = false;
        }
        if (!doneOnce) {
            shootColorWidget.withProperties(Map.of("colorWhenTrue", shootColor[shootColorNumber]));
            shootColorWidgetEntry.setBoolean(true);
            shootColorNumberLast = shootColorNumber;
            doneOnce = true;
        }
    }

    @Override
    public double getRPM() {
        return mEncoder.getVelocity();
    }

    public boolean atSpeed() {
        return Math.abs(requiredSpeed - getRPM()) < 50;

    }

    public void jogLeftMotor() {
        mPidController.setReference(.1, ControlType.kDutyCycle);
    }

    public double getLeftAmps() {
        return mLeftMotor.getOutputCurrent();
    }

    public double getRightAmps() {
        return 0;// mRightMotor.getOutputCurrent();
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
    }

    @Override
    public void stop() {
        mLeftMotor.set(0);
        // mRightMotor.set(0);
    }

    public void clearFaults() {
        mLeftMotor.clearFaults();

    }

    public int getFaults() {
        return mLeftMotor.getFaults();
    }

    private void setGains() {

        // PID coefficients
        kP = 6e-4;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.00015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        mPidController.setP(3e-4);
        mPidController.setI(0.00000);
        mPidController.setD(0.);
        mPidController.setIZone(1000.);
        mPidController.setFF(.00017);
        mPidController.setOutputRange(-1., 1.);

    }

}
