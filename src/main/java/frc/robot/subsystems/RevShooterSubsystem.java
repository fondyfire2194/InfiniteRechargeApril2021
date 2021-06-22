package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.SimpleCSVLogger;
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
    public double pset, iset, dset, ffset, izset;
    public boolean useCameraSpeed;
    public boolean useSetupSlider;
    public NetworkTableEntry shooterSpeed;

    public boolean startShooter;

    public int teleopSetupIndex = 3;
    public String[] teleopSetupPosition = new String[] { "InitLineStraightOn", "ShieldGenerator",
            "TrenchFrontOfControlPanel", "Trench Behind Control Panel", "Low Goal ", " ", " " };

    public double teleopSetupShooterSpeed;

    public SimpleCSVLogger simpleCSVLogger;

    public SimpleCSVLogger shootLogger;

    public SimpleCSVLogger shootSetupLogger;

    public boolean logSetup;

    public boolean shotInProgress;

    public PowerDistributionPanel pdp = new PowerDistributionPanel();

    private final int VELOCITY_SLOT = 0;
    /**
     * 8" diameter wheels = (8/12)*pi ft circ.
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
     * Following are arrays representing shoot mpersec and offset degrees for
     * distances from 2 to 10 meters in steps of 2 meters.
     * 
     * Calibrate both Limelight cursors. Turn off the Use Vision boolean. At 2
     * meters with shooter directly in front of target calibrate the first slider.
     * In the trench just in front of the control panel with the turret turned so
     * the shooter would hit the target, calibrate the second slider.
     * 
     * Turn on Use Vision and lock tilt and turret to Limeight. The SetUpShooter tab
     * has sliders for speed and offset. They are active when the "SpeedFromSlider"
     * indicator is on. It is toggled using the ToggleSpeedFromSlider button.
     * 
     * Slder range is 20 to 50 MPS. Offset slider range is -10 to +10 degrees
     * 
     * 
     * Starting at 2 meters fire cells and get the best speed and y offset to hit
     * center of target +/ 1/2 diameter of inner port .33/2 = .16 meters. Move back
     * to 4, 6, 8 and 10 meters and repeat.
     * 
     * Numbers will be entered in arays and interpolated so midway between 2
     * distance points, the speed will be the lower plus 1/2 the difference of the
     * range.
     * 
     * 
     */

    public double[] speedBreakMeters = new double[] { 2, 4, 6, 8, 10 };
    public double[] MPSFromCameraDistance = new double[] { 25, 30, 35, 40, 45 };
    public double[] tiltOffsetFromCameraDistance = new double[] { 0, 1, 2, 4, 7 };
    public double[] minOffsetFromCameraDistance = new double[] { 0, 0, 1, 2, 3 };

    private double minimumOffsetInRange = 2;

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
    public double cameraAngleCalculatedSpeed;
    public boolean hideSliders = Constants.isMatch;
    public boolean driverOKShoot;
    public boolean burnOK;
    public double shooterRecoverTime = .5;
    public boolean shootOne;
    public boolean endFile;
    public boolean endShootFile;
    public boolean isShooting;

    private boolean interpolateSpeed = false;
    private boolean interpolateOffsets = true;
    public boolean logTrigger;
    public double testVertOffset;
    public int itemsLogged;

    private double testDistance = 2;

    public boolean logSetupFileOpen;

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

        Arrays.asList(mLeftMotor, mRightMotor).forEach((SimableCANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake mode for faster stop
        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }
        mEncoder.setPositionConversionFactor(metersPerRev);
        mEncoder.setVelocityConversionFactor(metersPerRev / 60);

        if (!hideSliders) {

            shooterSpeed = Shuffleboard.getTab("SetupShooter").add("ShooterSpeed", 3).withWidget("Number Slider")
                    .withPosition(0, 3).withSize(4, 1).withProperties(Map.of("Min", 15, "Max", 50)).getEntry();

        }
        tuneGains();
        getGains();
        requiredMps = 23;
        shootOne = true;
        simpleCSVLogger = new SimpleCSVLogger();
        shootLogger = new SimpleCSVLogger();
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
        if (useSetupSlider) {
            requiredMps = shooterSpeed.getDouble(20);
        }
        if (DriverStation.getInstance().isOperatorControlEnabled()) {
            if (useCameraSpeed) {
                requiredMps = cameraCalculatedSpeed;
            }
        }

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
        return startShooter && Math.abs(requiredMps + getMPS()) < (requiredMps * .1);

    }

    public void jogLeftMotor() {
        mPidController.setReference(.1, ControlType.kDutyCycle);
    }

    public double getLeftAmps() {
        return mLeftMotor.getOutputCurrent();
    }

    public double getRightAmps() {
        return mRightMotor.getOutputCurrent();
    }

    @Override
    public void simulationPeriodic() {
        mSimulator.update();
    }

    @Override
    public void stop() {
        mLeftMotor.stopMotor();
        mRightMotor.stopMotor();
        startShooter = false;
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

    public double getBatteryVoltage() {

        return pdp.getVoltage();
    }

    public double[] calculateMPSFromDistance(double distance) {

        double[] temp = new double[] { 0, 0 };
        /**
         * The arrays have distances at which speed step changes
         * 
         * 
         */
        int distanceLength = speedBreakMeters.length;
        double minimumDistance = speedBreakMeters[0];
        double maximumDistance = speedBreakMeters[distanceLength - 1];

        SmartDashboard.putNumber("Amind", minimumDistance);
        SmartDashboard.putNumber("Amaxd", maximumDistance);

        double pu;
        double speedRange;
        double unitAdder;
        double distanceRange;
        double offsetRange;

        if (distance < minimumDistance)
            distance = minimumDistance;
        if (distance > maximumDistance)
            distance = maximumDistance;

        for (int i = 0; i < distanceLength - 1; i++) {
            if (distance >= speedBreakMeters[i] && distance < speedBreakMeters[i + 1]) {
                temp[0] = MPSFromCameraDistance[i];
                temp[1] = tiltOffsetFromCameraDistance[i];

                distanceRange = speedBreakMeters[i + 1] - speedBreakMeters[i];
                double distanceFromEndOfRange = speedBreakMeters[i + 1] - distance;

                pu = distanceFromEndOfRange / distanceRange;

                if (interpolateSpeed) {

                    speedRange = MPSFromCameraDistance[i + 1] - MPSFromCameraDistance[i];

                    unitAdder = speedRange * pu;
                    temp[0] += unitAdder;
                }

                if (interpolateOffsets) {
                    minimumOffsetInRange = minOffsetFromCameraDistance[i];
                    offsetRange = tiltOffsetFromCameraDistance[i] - minimumOffsetInRange;
                    SmartDashboard.putNumber("AMINOIR", minimumOffsetInRange);
                    SmartDashboard.putNumber("AI", i);
                    SmartDashboard.putNumber("AOFF", tiltOffsetFromCameraDistance[i]);

                    unitAdder = offsetRange * pu;
                    temp[1] += unitAdder;
                }
                break;
            }

        }
        return temp;
    }

    private void tuneGains() {
        fixedSettings();
        double f = Pref.getPref("sHff");
        double p = Pref.getPref("sHkp");
        double i = Pref.getPref("sHki");
        double d = Pref.getPref("sHkd");
        double iz = Pref.getPref("sHkiz");
        double acc = 1;

        calibratePID(p, i, d, f, iz, acc, VELOCITY_SLOT);

    }

    private void fixedSettings() {

        kMaxOutput = 1;
        kMinOutput = -1;

    }

    private void checkTune() {
        CANError burnError = CANError.kError;

        if (Pref.getPref("sHTune") == 5. && leftMotorConnected) {
            burnOK = false;
            burnError = mLeftMotor.burnFlash();
            burnOK = burnError == CANError.kOk;
            getGains();
        }

        tuneOn = Pref.getPref("sHTune") == 1. && leftMotorConnected;

        if (tuneOn && !lastTuneOn) {
            tuneGains();
            getGains();
            lastTuneOn = true;
        }
        if (lastTuneOn)
            lastTuneOn = tuneOn;

    }

    public void getGains() {
        ffset = mPidController.getFF(0);
        pset = mPidController.getP(0);
        iset = mPidController.getI(0);
        dset = mPidController.getD(0);
        izset = mPidController.getIZone();

    }

    public void shootOne() {
        shootOne = true;
    }

    public void shootAll() {
        shootOne = false;
    }

    public void setOKShootDriver() {
        driverOKShoot = true;
    }

    public void notOKShootDriver() {
        driverOKShoot = false;
    }

}
