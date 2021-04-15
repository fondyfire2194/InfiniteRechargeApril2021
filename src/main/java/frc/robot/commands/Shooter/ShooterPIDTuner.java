package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RevShooterSubsystem;

public class ShooterPIDTuner {
    private double p, i, d, f;
    private RevShooterSubsystem m_shooter;

    public ShooterPIDTuner(RevShooterSubsystem shooter) {
        m_shooter = shooter;

    }

    public void spinUpTune() {
        m_shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 1e-5);
    }

    public void HoldTune() {
        getPIDFromDashBoard();
        setPID();
    }

    public void getPIDFromDashBoard() {
        p = SmartDashboard.getNumber("shooter/P", 0.000145);
        i = SmartDashboard.getNumber("shooter/I", 1e-8);
        d = SmartDashboard.getNumber("shooter/D", 0);
        f = SmartDashboard.getNumber("shooter/F", 6.6774 * 1e-5);
    }

    public void setPID() {
        m_shooter.calibratePID(p, i, d, f);
    }
}
