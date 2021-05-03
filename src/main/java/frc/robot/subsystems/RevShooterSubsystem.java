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
import frc.robot.Pref;
import frc.robot.sim.ShooterSubsystem;

public class RevShooterSubsystem extends SubsystemBase implements ShooterSubsystem {
    public final SimableCANSparkMax mLeftMotor; // NOPMD
    private SimableCANSparkMax mRightMotor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mSimulator;
    public double requiredSpeedLast;
    public double requiredSpeed;
    public double shootTime;
    public double shootTimeRemaining;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    public static double kGearing = 1;
    public static double kInertia = 0.008;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private SimpleWidget shootColorWidget;
    private NetworkTableEntry shootColorWidgetEntry;
    private boolean doneOnce;
    public double cameraCalculatedSpeed;
    public boolean useCameraSpeed;
    private final int VELOCITY_SLOT = 0;

    /**
     * 
     * 
     * following is array representing shoot speeds for distances from 3 to 13
     * meters or 10 meters
     *
     * 10 meters with steps of 1 meter is 10 steps or 40 inches.
     * 
     * we can measure every meter and out results in array and then interpolate.
     */
    public double[] speedFromCamera = new double[] { 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500 };

    public String[] shootColor = { "red", "yellow", "green" };
    public int shootColorNumber;
    private int shootColorNumberLast = 1;
    public double startDistance;
    public double calculatedCameraDistance;

    public boolean tuneOn = false;
    

    public RevShooterSubsystem() {
        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeftMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(5.);
        mLeftMotor.setIdleMode(IdleMode.kBrake);

        mRightMotor.restoreFactoryDefaults();
        mRightMotor.follow(mLeftMotor);
        mRightMotor.setIdleMode(IdleMode.kBrake);

        int stallLimit = 45;
        int freeLimit = 55;
        int limitRPM = 0;

        mLeftMotor.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

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

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        mPidController.setIAccum(0);
        mPidController.setP(p, slotNumber);
        mPidController.setI(i, slotNumber);
        mPidController.setD(d, slotNumber);
        mPidController.setFF(f, slotNumber);
        mPidController.setIZone(kIz, slotNumber);
        mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
        mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);

    }

    @Override
    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    @Override
    public void spinAtRpm(double rpm) {
        requiredSpeed = rpm;
        // SmartDashboard.putNumber("SHMXO", mPidController.getOutputMax(VELOCITY_SLOT));
        // SmartDashboard.putNumber("SHMINO", mPidController.getOutputMin(VELOCITY_SLOT));
        // SmartDashboard.putNumber("SHVP", mPidController.getP(VELOCITY_SLOT));
        // SmartDashboard.putNumber("SHVFF", mPidController.getFF(VELOCITY_SLOT));

        // SmartDashboard.putNumber("SHVI", mPidController.getI(VELOCITY_SLOT));
        // SmartDashboard.putNumber("SHVIZ", mPidController.getIZone(VELOCITY_SLOT));
        // SmartDashboard.putNumber("SHVEL", rpm);

        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public void moveManually(double speed) {
        mLeftMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        tuneOn = Pref.getPref("sHTune") != 0.;

        if (tuneOn)
            tuneGains();

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

        mRightMotor.set(0);
    }

    public void clearLeftFaults() {
        mLeftMotor.clearFaults();

    }

    public void clearRightFaults() {

        mRightMotor.clearFaults();
    }

    public void clearFaults() {
        clearLeftFaults();
        clearRightFaults();
    }

    public int getLeftFaults() {
        return mLeftMotor.getFaults();
    }

    public int getRightFaults() {
        return mLeftMotor.getFaults();
    }

    public int getFaults() {
        return mLeftMotor.getFaults() + mRightMotor.getFaults();
    }

    private void setGains() {
        fixedSettings();
        maxRPM = 5700;
        kFF = .000085;
        kP = .000;
        kI = 0.0;
        kD = 0;
        kIz = 2;

        calibratePID(kP, kI, kD, kFF, kIz, VELOCITY_SLOT);
    }

    private void tuneGains() {
        fixedSettings();
        double f = Pref.getPref("sHFf");
        double p = Pref.getPref("sHKp");
        double i = Pref.getPref("sHKi");
        double d = Pref.getPref("sHKd");
        double iz = Pref.getPref("sHKiz");

        calibratePID(p, i, d, f, iz, VELOCITY_SLOT);
    }

    private void fixedSettings() {
        kMaxOutput = 1;
        kMinOutput = -1;

        maxRPM = 11000;// not used
        allowedErr = 1000;

    }
}
