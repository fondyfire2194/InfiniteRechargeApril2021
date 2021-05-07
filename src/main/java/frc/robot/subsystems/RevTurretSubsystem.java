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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.Pref;
import frc.robot.sim.ElevatorSubsystem;

public class RevTurretSubsystem extends SubsystemBase implements ElevatorSubsystem {

    private static final double DEG_PER_MOTOR_REV = HoodedShooterConstants.TURRET_DEG_PER_MOTOR_REV;
    public final int POSITION_SLOT = 0;
    public final int VISION_SLOT = 1;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kAcc;
    public double lastkP, lastkI, lastkD, lastkIz, lastkFF, lastkMaxOutput, lastkMinOutput, lastkAcc;

    private final SimableCANSparkMax m_motor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mElevatorSim;
    public double visionCorrection;
    public double targetAngle;
    private double inPositionBandwidth = .25;
    public double targetHorizontalOffset;
    public boolean validTargetSeen;
    public double adjustedTargetAngle;

    public boolean tuneOn = false;
    public boolean lastTuneOn;
    private int loopCtr;

    public boolean turretMotorConnected;
    private double startTime;
    private double endTime;

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

        if (RobotBase.isReal()) {
            m_motor.setSmartCurrentLimit(5);

            mEncoder.setPositionConversionFactor(DEG_PER_MOTOR_REV);
            mEncoder.setVelocityConversionFactor(DEG_PER_MOTOR_REV / 60);
        } else

        {
            mEncoder.setPositionConversionFactor(1); // // HoodedShooterConstants.TURRET_ENCODER_DEG_PER_REV);
            mPidController.setP(.1, 0);

        }

        m_motor.setIdleMode(IdleMode.kBrake);

        // if (RobotBase.isSimulation())
        // mPidController.setP(0.16);

        mEncoder.setPosition(0);
        targetAngle = 0;

        // setSoftwareLimits();

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
        loopCtr++;

        tuneOn = Pref.getPref("tURTune") != 0.;

        if (tuneOn && !lastTuneOn) {
            startTime = Timer.getFPGATimestamp();
            tuneGains();
            lastTuneOn = true;
            endTime = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("TUGT", endTime - startTime);
        }

        if (lastTuneOn)
            lastTuneOn = tuneOn;

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
        mPidController.setReference(angle, ControlType.kPosition, POSITION_SLOT);

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
        m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_motor.setIdleMode(IdleMode.kBrake);
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

    public void calibratePID(double p, double i, double d, double acc, double kIz, int slotNumber) {

        if (p != lastkP) {
            mPidController.setP(p, slotNumber);
            lastkP = p;// mPidController.getP(slotNumber);

        }
        if (i != lastkI) {
            mPidController.setI(i, slotNumber);
            lastkI = i;// mPidController.getI(slotNumber);
        }

        if (d != lastkD) {
            mPidController.setD(d, slotNumber);
            lastkD = d;// mPidController.getD(slotNumber);
        }
        if (acc != lastkAcc) {
            m_motor.setClosedLoopRampRate(acc);
            lastkAcc = acc;// m_motor.getClosedLoopRampRate();
        }
        if (kIz != lastkIz) {
            mPidController.setIZone(kIz, slotNumber);
    
                lastkIz = kIz;// mPidController.getIZone(slotNumber);
        }
        if (kMinOutput != lastkMinOutput || kMaxOutput != lastkMaxOutput) {
            mPidController.setOutputRange(kMinOutput, kMaxOutput, slotNumber);
            lastkMinOutput = kMinOutput;
            lastkMaxOutput = kMaxOutput;
        }

    }

    private void setGains() {
 /**
  * Using the position mode doesn't require feedforward. 
  */
        fixedSettings();

        kP = .0176;
        kI = 0.000012;
        kD = .000001;
        kIz = 1;

        kAcc = 500;

        // set PID coefficients
        calibratePID(kP, kI, kD, kAcc, kIz, POSITION_SLOT);
        calibratePID(kP, kI, kD, kAcc, kIz, VISION_SLOT);

    }

    private void tuneGains() {
        // fixedSettings();

        double p = Pref.getPref("tURKp");
        double i = Pref.getPref("tURKi");
        double d = Pref.getPref("tURKd");
        double iz = Pref.getPref("tURKiz");
        kAcc = Pref.getPref("tURAcc");

        calibratePID(p, i, d, kAcc, iz, POSITION_SLOT);
        calibratePID(p, i, d, kAcc, iz, VISION_SLOT);
    }

    private void fixedSettings() {
        kFF = .0000;//
        mPidController.setFF(kFF);
        kMaxOutput = 1;
        kMinOutput = -1;

    }

    public void clearFaults() {
        m_motor.clearFaults();
    }

    public int getFaults() {
        return m_motor.getFaults();
    }

    public void aimFurtherLeft(double angle) {
        targetHorizontalOffset -= angle;
    }

    public void aimFurtherRight(double angle) {
        targetHorizontalOffset += angle;
    }

    public void aimCenter() {
        targetHorizontalOffset = 0;
    }

}
