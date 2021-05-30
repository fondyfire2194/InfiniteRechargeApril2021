package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTurretSubsystem extends SubsystemBase implements ElevatorSubsystem {

    private static final double DEG_PER_MOTOR_REV = HoodedShooterConstants.TURRET_DEG_PER_MOTOR_REV;
    private static final int SMART_MOTION_SLOT = 1;
    public final int POSITION_SLOT = 0;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastmaxRPM, lastmaxVel,
            lastminVel, lastmaxAcc, lastallowedErr;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    public final CANPIDController mPidController;
    private ISimWrapper mElevatorSim;
    public double targetAngle;
    private double inPositionBandwidth = .25;
    public double targetHorizontalOffset;
    public boolean validTargetSeen;
    public double adjustedTargetAngle;

    public boolean tuneOn = false;
    public boolean lastTuneOn;

    public boolean turretMotorConnected;

    private double maxAdjustShoot = .5;

    public RevTurretSubsystem() {
        m_motor = new SimableCANSparkMax(CANConstants.TURRET_ROTATE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
        mEncoder = m_motor.getEncoder();
        mPidController = m_motor.getPIDController();
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        m_motor.setOpenLoopRampRate(5);

        mEncoder.setPosition(0);
        aimCenter();

        setGains();
        setSoftwareLimits();

        if (RobotBase.isReal()) {
            // m_motor.setSmartCurrentLimit(5);

            mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);
            mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);
        } else

        {
            mEncoder.setPositionConversionFactor(1.421); // // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV);
            mPidController.setP(.1, 0);

        }

        m_motor.setIdleMode(IdleMode.kBrake);

        // if (RobotBase.isSimulation())
        // mPidController.setP(0.16);

        mEncoder.setPosition(0);
        targetAngle = 0;

        if (RobotBase.isSimulation())

        {
            ElevatorSimConstants.kCarriageMass = .001;
            ElevatorSimConstants.kElevatorGearing = 1;
            ElevatorSimConstants.kMaxElevatorHeight = 100;
            ElevatorSimConstants.kMinElevatorHeight = -110;
            ElevatorSimConstants.kElevatorGearbox = DCMotor.getNeo550(1);

            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(m_motor), RevEncoderSimWrapper.create(m_motor));

        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        tuneOn = Pref.getPref("tURTune") != 0.;

        checkTune();

        if (DriverStation.getInstance().isDisabled())
            targetAngle = getAngle();

    }

    public boolean checkCAN() {
        return turretMotorConnected = m_motor.getFirmwareVersion() != 0;
    }

    @Override
    public void close() {
        m_motor.close();
    }

    @Override
    public void moveManually(double speed) {
        m_motor.set(speed);
        targetAngle = getAngle();
    }

    @Override
    public void goToPosition(double angle) {
        mPidController.setReference(angle, ControlType.kPosition, POSITION_SLOT);

    }

    public void positionTurret(double angle, int slotNumber) {
        mPidController.setReference(angle, ControlType.kPosition, slotNumber);

    }

    @Override
    public void goToPositionMotionMagic(double angle) {
        mPidController.setReference(angle, ControlType.kSmartMotion, SMART_MOTION_SLOT);
        SmartDashboard.putNumber("TUEP", angle);

    }

    public void resetAngle(double angle) {
        mEncoder.setPosition(angle);
        mPidController.setIAccum(0);
        targetAngle = angle;
    }

    public double getOut() {
        return m_motor.get();
    }

    public double getAmps() {
        return m_motor.getOutputCurrent();
    }

    public double getSpeed() {
        return mEncoder.getVelocity();
    }

    public void setSoftwareLimits() {
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kForward,
                (float) HoodedShooterConstants.TURRET_MAX_ANGLE);
        m_motor.setSoftLimit(SimableCANSparkMax.SoftLimitDirection.kReverse,
                (float) HoodedShooterConstants.TURRET_MIN_ANGLE);
        enableSofLimits(true);
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void enableSofLimits(boolean on) {
        SmartDashboard.putBoolean("TUSL", on);
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, on);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, on);
    }

    public boolean getSoftwareLimitsEnabled() {
        return m_motor.isSoftLimitEnabled(SoftLimitDirection.kForward)
                || m_motor.isSoftLimitEnabled(SoftLimitDirection.kReverse);
    }

    public boolean isBrake() {
        return m_motor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean onPlusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean onMinusSoftwareLimit() {
        return m_motor.getFault(FaultID.kSoftLimitRev);
    }

    public void aimFurtherLeft(double angle) {
        if (targetHorizontalOffset > -maxAdjustShoot)
            targetHorizontalOffset -= angle;
    }

    public void aimFurtherRight(double angle) {
        if (targetHorizontalOffset < maxAdjustShoot)
            targetHorizontalOffset += angle;
    }

    public void aimCenter() {
        targetHorizontalOffset = 0;
    }

    @Override
    public boolean isAtHeight(double angle, double allowableError) {
        return Math.abs(angle - getAngle()) < allowableError;
    }

    public boolean atTargetAngle() {
        return Math.abs(targetAngle - getAngle()) < inPositionBandwidth;
    }

    public double getAngle() {
        return mEncoder.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        mElevatorSim.update();
    }

    @Override
    public void stop() {
        m_motor.set(0);
    }

    public void moveManuallyVelocity(double speed) {
        targetAngle = getAngle();
        mPidController.setReference(speed, ControlType.kVelocity, POSITION_SLOT);
    }

    public double getIaccum() {
        return mPidController.getIAccum();
    }

    public void calibratePID(final double p, final double i, final double d, final double f, final double kIz,
            int slotNumber) {
        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = mPidController.getP(slotNumber);

        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = mPidController.getI(slotNumber);
        }

        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = mPidController.getD(slotNumber);
        }
        if (f != lastkFF) {
            mPidController.setFF(f, slotNumber);
            lastkFF = f;
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
            lastkIz = mPidController.getIZone(slotNumber);
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }
        if (slotNumber == SMART_MOTION_SLOT) {
            if (lastmaxAcc != maxAcc) {
                mPidController.setSmartMotionMaxAccel(maxAcc, slotNumber);
                lastmaxAcc = maxAcc;
            }

            if (lastmaxVel != maxVel) {
                mPidController.setSmartMotionMaxVelocity(maxAcc, slotNumber);
                lastmaxVel = maxVel;
            }

            if (lastallowedErr != allowedErr) {
                mPidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotNumber);
                lastallowedErr = allowedErr;

            }

        }

    }

    private void setGains() {

        fixedSettings();

        kP = .00018;
        kI = 0.001;
        kD = .0002;
        kIz = 1;

        maxAcc = 850;
        maxVel = 2500;

        // set PID coefficients
        calibratePID(kP, kI, kD, kFF, kIz, SMART_MOTION_SLOT);

    }

    private void tuneGains() {
        fixedSettings();

        double p = Pref.getPref("tURKp");
        double i = Pref.getPref("tURKi");
        double d = Pref.getPref("tURKd");
        double iz = Pref.getPref("tURKiz");
        maxVel = Pref.getPref("tURMaxV");
        maxAcc = Pref.getPref("tURMaxA");

        calibratePID(p, i, d, kFF, iz, SMART_MOTION_SLOT);

    }

    private void fixedSettings() {
        kFF = .00007;// 10000 rpm = 10000 * 1.42 deg / rev =14,200 1/14200 = 7e-5
        kMaxOutput = .75;
        kMinOutput = -.75;
        maxRPM = 11000;// not used
        allowedErr = .01;
    }

    private void checkTune() {
        tuneOn = Pref.getPref("tURTune") != 0.;

        if (tuneOn && !lastTuneOn) {

            tuneGains();
            lastTuneOn = true;
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;
    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

}
