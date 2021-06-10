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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants;
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
    public boolean startShooter;

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

    public double[] speedBreakAngles = new double[] { 30, 20, 12, 6, 0 };
    public double[] speeedBreakOffset = new double[] { 5, 4, 2, 0, 0 };
    public double[] speedMPS = new double[] { 35, 30, 25, 20 };

    public double[] shooterMPSfromCameraAngle = new double[] { 50, 47.5, 45, 42.5, 40, 37.5, 35, 32.5, 30, 27.5, 25 };

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

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(mLeftMotor, mRightMotor)
                .forEach((SimableCANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        if (RobotBase.isSimulation()) {
            mSimulator = new FlywheelSimWrapper(FlywheelSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeftMotor), RevEncoderSimWrapper.create(mLeftMotor));
        }
        mEncoder.setPositionConversionFactor(metersPerRev);
        mEncoder.setVelocityConversionFactor(metersPerRev / 60);

        if (!hideSliders) {

            setupVertOffset = Shuffleboard.getTab("SetupShooter").add("SetupVertOffset", 0).withWidget("Number Slider")
                    .withPosition(4, 3).withSize(2, 1).withProperties(Map.of("Min", -10, "Max", 10)).getEntry();

            shooterSpeed = Shuffleboard.getTab("SetupShooter").add("ShooterSpeed", 3).withWidget("Number Slider")
                    .withPosition(0, 3).withSize(4, 1).withProperties(Map.of("Min", 0, "Max", 45)).getEntry();
        }
        // tuneGains();
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

        calculateMPSandYOffset(shooterSpeed.getDouble(0));

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

        double baseSpeed = 0;
        double offset = 0;
        double angleRange = 0;
        double offsetRange = 0;
        double offsetBaseValue = 0;
        double rangeBaseAngle = 0;
        double offsetSlope = 0;
        double distanceIntoRange = 0;

        int j = speedBreakAngles.length - 1;
        SmartDashboard.putNumber("ILGHT", j);
        for (int i = 0; i < j; i++) {
            if (angle >= speedBreakAngles[i + 1] && angle <= speedBreakAngles[i]) {
                baseSpeed = speedMPS[i];
                offsetBaseValue = speeedBreakOffset[i];
                offsetRange = speeedBreakOffset[i + 1] - speeedBreakOffset[i];
                angleRange = speedBreakAngles[i + 1] - speedBreakAngles[i];
                distanceIntoRange = angle - speedBreakAngles[i];
                SmartDashboard.putNumber("IDIR", distanceIntoRange);
                offsetSlope = offsetRange / angleRange;
                SmartDashboard.putNumber("IOFFRGE", offsetRange);
                offset = offsetBaseValue + offsetSlope * distanceIntoRange;
                SmartDashboard.putNumber("I", i);

                break;
            }
        }

        temp[1] = offset;
        temp[0] = baseSpeed;

        SmartDashboard.putNumber("IOF", temp[1]);
        SmartDashboard.putNumber("ISPD", temp[0]);

        return temp;

    }

    public double calculateMPSFromAngle(double angle) {
        /**
         * Speed will decrease as angle increases Need to find the base of the speed
         * range for the angle then the difference between that and the next speed and
         * subtract the amount into that range from the base.
         * 
         * speedBaseAngle = 8;speedMaxAngle = 30;
         * 
         * shooterMPSfromCameraAngle = 50, 47.5,45,42.5 40, 37.5,35, 30, 25, 20, 15, 12,
         * 18, 8, 5
         * 
         * So an angle of 15
         *
         */

        if (angle < speedBaseAngle)
            angle = speedBaseAngle;
        if (angle > speedMaxAngle)
            angle = speedMaxAngle;

        // find index into array since it doesn't start at 0

        double tempAngle = angle - speedBaseAngle;

        int baseI = (int) tempAngle;
        double base = (double) baseI;

        double rem = tempAngle - base;

        double baseSpeed = shooterMPSfromCameraAngle[baseI];
        double upperSpeed = shooterMPSfromCameraAngle[baseI + 1];

        double speedRange = baseSpeed - upperSpeed;

        double speedAdder = speedRange * rem;

        cameraAngleCalculatedSpeed = baseSpeed - speedAdder;

        useCameraAngleSpeed = true;

        return cameraAngleCalculatedSpeed;

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
        SmartDashboard.putBoolean("ShooterBurnOK", false);
        if (Pref.getPref("sHTune") == 5.) {
            burnError = mLeftMotor.burnFlash();
            SmartDashboard.putBoolean("ShooterBurnOK", burnError == CANError.kOk);
        }



        tuneOn = Pref.getPref("sHTune") == 1.;

        if (tuneOn && !lastTuneOn) {
            tuneGains();
            lastTuneOn = true;
        }
        if (lastTuneOn)
            lastTuneOn = tuneOn;

    }
}
