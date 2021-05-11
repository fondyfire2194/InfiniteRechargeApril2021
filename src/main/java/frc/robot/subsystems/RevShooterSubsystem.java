package frc.robot.subsystems;

import java.util.Arrays;

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

import edu.wpi.first.wpilibj.RobotBase;
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
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, acc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastAcc;
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
    public double startDistance;
    public double calculatedCameraDistance;

    public boolean tuneOn = false;
    public boolean lastTuneOn;
    public boolean leftMotorConnected;
    public boolean rightMotorConnected;
    public boolean allConnected;
    public boolean cameraSpeedBypassed;

    public RevShooterSubsystem() {
        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeftMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(5.);

        mRightMotor.restoreFactoryDefaults();
        mRightMotor.follow(mLeftMotor, true);

        Arrays.asList(mLeftMotor, mRightMotor).forEach((SimableCANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }
        requiredSpeed = 2500;
        setGains();

    }

    @Override
    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    @Override
    public void spinAtRpm(double rpm) {
        requiredSpeed = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public void runShooter() {

        spinAtRpm(requiredSpeed);
    }

    public void moveManually(double speed) {
        mLeftMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        tuneOn = Pref.getPref("sHTune") != 0.;

        if (tuneOn && !lastTuneOn) {
            tuneGains();
            lastTuneOn = true;
        }
        if (lastTuneOn)
            lastTuneOn = tuneOn;

        if (useCameraSpeed && !cameraSpeedBypassed)
            requiredSpeed = cameraCalculatedSpeed;

    }

    public boolean checkCAN() {
        leftMotorConnected = mLeftMotor.getFirmwareVersion() != 0;
        rightMotorConnected = mRightMotor.getFirmwareVersion() != 0;
        allConnected = leftMotorConnected && rightMotorConnected;
        return allConnected;
    }

    @Override
    public double getRPM() {
        return mEncoder.getVelocity();
    }

    public boolean atSpeed() {
        return requiredSpeed > 0 && Math.abs(requiredSpeed - getRPM()) < requiredSpeed * .1;

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

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            double acc, int slotNumber) {

        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = p;
        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = i;
        }
        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = d;
        }

        if (f != lastkFF) {
            mPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIz = kIz;
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (acc != lastAcc) {
            mLeftMotor.setClosedLoopRampRate(acc);
            lastAcc = acc;
        }

    }

    public double calculateSpeedFromDistance(double distance) {
        SmartDashboard.putNumber("D", distance);
        calculatedCameraDistance = distance;
        if (distance >= 3 && distance <= 13) {
            // subtract base distance of 3 meters
            double baseDistance = 3;
            double tempDistance = distance - baseDistance;

            int baseI = (int) tempDistance;
            double base = (double) baseI;

            double rem = tempDistance - base;

            SmartDashboard.putNumber("BI", baseI);
            SmartDashboard.putNumber("B", base);
            SmartDashboard.putNumber("BR", rem);

            double baseSpeed = speedFromCamera[baseI];
            double upperSpeed = speedFromCamera[baseI + 1];

            double speedRange = upperSpeed - baseSpeed;

            double speedAdder = speedRange * rem;

            cameraCalculatedSpeed = baseSpeed + speedAdder;

            useCameraSpeed = true;

        } else {

            cameraCalculatedSpeed = 2500;
        }
        return cameraCalculatedSpeed;

    }

    private void setGains() {
        fixedSettings();

        kFF = .00017;
        kP = 3e-4;
        kI = 0.0001;
        kD = 0;
        kIz = 500;
        acc = 500;

        calibratePID(kP, kI, kD, kFF, kIz, acc, VELOCITY_SLOT);
    }

    private void tuneGains() {
        fixedSettings();
        double f = Pref.getPref("sHFf");
        double p = Pref.getPref("sHKp");
        double i = Pref.getPref("sHKi");
        double d = Pref.getPref("sHKd");
        double iz = Pref.getPref("sHKiz");
        double acc = Pref.getPref("sHKacc");

        calibratePID(p, i, d, f, iz, acc, VELOCITY_SLOT);
    }

    private void fixedSettings() {

        kMaxOutput = 1;
        kMinOutput = -1;

    }
}
