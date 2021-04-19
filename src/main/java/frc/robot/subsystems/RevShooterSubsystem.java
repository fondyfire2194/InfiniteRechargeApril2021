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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Robot;
import frc.robot.sim.ShooterSubsystem;

public class RevShooterSubsystem extends SubsystemBase implements ShooterSubsystem {
    public final SimableCANSparkMax mLeftMotor; // NOPMD
    private final SimableCANSparkMax mRightMotor; // NOPMD
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
    public double startDistance;
    public double calculatedCameraDistance;

    private boolean tuneOn = false;

    public RevShooterSubsystem() {
        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mRightMotor.follow(mLeftMotor);

        mEncoder = mLeftMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mRightMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(5.);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        kMaxOutput = 1;
        kMinOutput = -1;
        mPidController.setOutputRange(kMinOutput, kMaxOutput);

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }

        shootColorWidget = Shuffleboard.getTab("Competition").add("ShootColor", false).withWidget("Boolean Box")
                .withPosition(0, 0).withSize(2, 1).withProperties(Map.of("colorWhenFalse", "black"));
        shootColorWidgetEntry = shootColorWidget.getEntry();
        shootColorWidgetEntry.getBoolean(false);

        if (!tuneOn) {
            setGains();
        }

    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz) {
        mPidController.setIAccum(0);
        mPidController.setP(p);
        mPidController.setI(i);
        mPidController.setD(d);
        mPidController.setFF(f);
        mPidController.setIZone(kIz);
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

        if (tuneOn)
            tuneGains();

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
        maxRPM = 5700;
        kP = .002;
        kI = 0.01;
        kD = 0;
        kIz = 2;
        kFF = 2e-4;

        calibratePID(kP, kI, kD, kFF, kIz);
    }

    private void tuneGains() {

        double p = Robot.tuneValues.getEntry("kP").getDouble(0.002);
        double i = Robot.tuneValues.getEntry("kI").getDouble(0.01);
        double d = Robot.tuneValues.getEntry("kD").getDouble(0);
        double ff = Robot.tuneValues.getEntry("kFF").getDouble(2);
        double iz = Robot.tuneValues.getEntry("kIZ").getDouble(2e-4);
     
        calibratePID(p, i, d, ff, iz);
    }

}
