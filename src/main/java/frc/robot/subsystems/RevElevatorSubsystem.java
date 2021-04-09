package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.SimableCANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CANConstants;
import frc.robot.sim.ElevatorSubsystem;

import org.snobotv2.module_wrappers.rev.RevEncoderSimWrapper;
import org.snobotv2.module_wrappers.rev.RevMotorControllerSimWrapper;
import org.snobotv2.sim_wrappers.ElevatorSimWrapper;
import org.snobotv2.sim_wrappers.ISimWrapper;

public class RevElevatorSubsystem extends SubsystemBase implements ElevatorSubsystem
{
    private static final double GRAVITY_COMPENSATION_VOLTS = 0.85;
    private static final double TICKS_PER_METER = 4096;
    private static final int POSITION_SLOT = 0;
    private static final int SMART_MOTION_SLOT = 1;

    private final SimableCANSparkMax mLeadMotor; // NOPMD
//    private final SimableCANSparkMax mFollowerMotor; // NOPMD
    private final CANEncoder mEncoder;
    private final CANPIDController mPidController;
    private ISimWrapper mElevatorSim;
    private int n;

    public RevElevatorSubsystem()
    {
        mLeadMotor = new SimableCANSparkMax(18, CANSparkMaxLowLevel.MotorType.kBrushless);

        mEncoder = mLeadMotor.getEncoder();
        mPidController = mLeadMotor.getPIDController();

        mEncoder.setPositionConversionFactor(TICKS_PER_METER);

        mPidController.setP(0.16);

        if (Robot.isSimulation())
        {
            
            mElevatorSim = new ElevatorSimWrapper(ElevatorSimConstants.createSim(),
                    new RevMotorControllerSimWrapper(mLeadMotor),
                    RevEncoderSimWrapper.create(mLeadMotor));
         
                
        }
    }

    @Override
    public void close()
    {
        mLeadMotor.close();
       
    }

    @Override
    public void moveManually(double speed)
    {
        mLeadMotor.set(speed);
    }

    @Override
    public void goToPosition(double inches)
    {
        double meters = Units.inchesToMeters(inches);
        SmartDashboard.putNumber("elmd", meters);
        n++;
        SmartDashboard.putNumber("eln", n);
        mPidController.setReference(meters, ControlType.kPosition, POSITION_SLOT, GRAVITY_COMPENSATION_VOLTS, CANPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void goToPositionMotionMagic(double inches)
    {
        double meters = Units.inchesToMeters(inches);
        mPidController.setReference(meters, ControlType.kSmartMotion, SMART_MOTION_SLOT);
    }

    @Override
    public boolean isAtHeight(double inches, double allowableError)
    {
        return Math.abs(inches - getHeightInches()) < allowableError;
    }

    @Override
    public double getHeightInches()
    {
        return Units.metersToInches(mEncoder.getPosition());
    }

    @Override
    public void simulationPeriodic()

    {

        mElevatorSim.update();
     
        
    
    }

    @Override
    public void stop()
    {
        mLeadMotor.set(0);
    }
}
