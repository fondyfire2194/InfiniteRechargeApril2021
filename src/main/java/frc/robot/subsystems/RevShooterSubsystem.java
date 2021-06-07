package frc.robot.subsystems;

import java.util.Arrays;
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
    private final CANEncoder mRightEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mSimulator;
    public double requiredMpsLast;
    public double requiredMps;
    public double shotDistance;
    public double shootTime;
    public double shootTimeRemaining;
    public static DCMotor kGearbox = DCMotor.getNeo550(2);
    public static double kGearing = 1;
    public static double kInertia = 0.008;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, acc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastAcc;
    public double cameraCalculatedSpeed;
    public boolean useCameraSpeed;
    public boolean useCameraAngleSpeed;
    public NetworkTableEntry shooterSpeed;
    public NetworkTableEntry setupVertOffset;
    public double useSetupOffset;
    public boolean useSetupSlider;
    public boolean useSetupVetOffset;

    private final int VELOCITY_SLOT = 0;
    /**
     * 8" diameter wheels = (8/12)*pi ft circ.
     * 
     * 
     * 
     * 
     * so circ = (2 *pi)/3 = 2.1 ft = .638 meters per rev max speed 80 revs per sec
     * so about 50 meters per sec 3000 per minute = max Angle range is 30 to around
     * 1
     */
    public final double metersPerRev = .638;

    /**
     * 
     * https://www.omnicalculator.com/physics/projectile-motion
     * 
     * following is array representing shoot mpersec for distances from 2 to 14
     * meters or 10 meters
     *
     * 10 meters with steps of 1 meter is 10 steps or 40 inches per step.
     * 
     * we can measure every meter, put results in array and then interpolate.
     */
    private int speedBaseAngle = 0;
    private int speedMaxAngle = 30;
    private double offsetMax = 5;

    public double[] speedBreakAngles = new double[] { 0, 6, 12, 18, 24, 30, 32 };

    public double[] speedMPS = new double[] { 5, 7, 15, 21, 32, 40, 42 };

    public String[] shootColor = { "red", "yellow", "green" };
    public int shootColorNumber;
    public double startDistance;
    public double calculatedCameraDistance;
    public double innerPortFloorDistance;

    public boolean tuneOn = false;
    public boolean lastTuneOn;
    public boolean leftMotorConnected;
    public boolean rightMotorConnected;
    public boolean allConnected;

    public RevShooterSubsystem() {

        mLeftMotor = new SimableCANSparkMax(CANConstants.LEFT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mRightMotor = new SimableCANSparkMax(CANConstants.RIGHT_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeftMotor.getEncoder();
        mRightEncoder = mRightMotor.getEncoder();
        mPidController = mLeftMotor.getPIDController();
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setOpenLoopRampRate(5.);
        mLeftMotor.setClosedLoopRampRate(1.);

        mRightMotor.restoreFactoryDefaults();
        mRightMotor.follow(mLeftMotor, true);

        // Arrays.asList(mLeftMotor, mRightMotor).forEach((SimableCANSparkMax spark) ->
        // spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }
        mEncoder.setPositionConversionFactor(metersPerRev);
        mEncoder.setVelocityConversionFactor(metersPerRev / 60);

        shooterSpeed = Shuffleboard.getTab("SetupShooter").addPersistent("ShooterSpeed", 3).withWidget("Number Slider")
                .withPosition(0, 3).withSize(5, 1).withProperties(Map.of("Min", 0, "Max", 50)).getEntry();

        setupVertOffset = Shuffleboard.getTab("SetupShooter").add("SetupVertOffset", 0).withWidget("Number Slider")
                .withPosition(5, 3).withSize(2, 1).withProperties(Map.of("Min", 0, "Max", 5)).getEntry();

        tuneGains();
        requiredMps = 12;
    }

    @Override
    public void close() {
        mLeftMotor.close();
        mRightMotor.close();
    }

    @Override
    public void spinAtRpm(double rpm) {
        requiredMps = rpm;
        mPidController.setReference(rpm, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public void spinAtMetersPerSec(double metersPerSec) {
        mPidController.setReference(metersPerSec, ControlType.kVelocity, VELOCITY_SLOT);
    }

    public void runShooter() {

        spinAtMetersPerSec(-requiredMps);
    }

    public void moveManually(double speed) {
        mLeftMotor.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkTune();

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

    public double getRightRPM() {
        return mRightEncoder.getVelocityConversionFactor();
    }

    public double getMPS() {
        return mEncoder.getVelocity();
    }

    public boolean atSpeed() {
        return requiredMps > 0 && Math.abs(requiredMps - getMPS()) < requiredMps * .1;

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

    public double getLeftPctOut() {
        return mLeftMotor.get();
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

    public double[] calculateMPSandYOffset(double angle) {
        /**
         * Find the base speed of the range the angle falls into
         * 
         * The Y offset starts at 0 and increase to the maximum offset with the angle
         */

        double[] temp = { 0, 0 };
        if (angle <= speedBaseAngle)
            angle = speedBaseAngle;
        if (angle >= speedMaxAngle)
            angle = speedMaxAngle;
        double speed = 0;
        double offset = 0;
        double angleRange = 0;
        double rangeBaseAngle = 0;
        int j = speedBreakAngles.length;
        for (int i = 0; i < j; i++) {
            if (angle >= speedBreakAngles[i] && angle < speedBreakAngles[i + 1]) {
                speed = speedMPS[i];
                rangeBaseAngle = speedBreakAngles[i];
                angleRange = speedBreakAngles[i + 1] - speedBreakAngles[i];
                break;
            }
        }
        temp[0] = speed;
        offset = (offsetMax * (angle - rangeBaseAngle)) / angleRange;
        temp[1] = offset;

        return temp;

    }

    private void setGains() {
        fixedSettings();

        kFF = .0003;// 1/3000
        kP = 3e-4;
        kI = 0.0001;
        kD = 0;
        kIz = 500;
        acc = 500;

        calibratePID(kP, kI, kD, kFF, kIz, acc, VELOCITY_SLOT);
    }

    private void tuneGains() {
        fixedSettings();
        double f = Pref.getPref("sHff");
        double p = Pref.getPref("sHkp");
        double i = Pref.getPref("sHki");
        double d = Pref.getPref("sHkd");
        double iz = Pref.getPref("sHkiz");
        double acc = Pref.getPref("sHkacc");

        calibratePID(p, i, d, f, iz, acc, VELOCITY_SLOT);
    }

    private void fixedSettings() {

        kMaxOutput = 1;
        kMinOutput = -1;

    }

    private void checkTune() {
        tuneOn = Pref.getPref("sHTune") != 0.;

        if (tuneOn && !lastTuneOn) {
            tuneGains();
            lastTuneOn = true;
        }
        if (lastTuneOn)
            lastTuneOn = tuneOn;

    }
}
