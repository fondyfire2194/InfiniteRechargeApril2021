// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

/** Add your docs here. */
public class SimLimelight {

    private final RevTurretSubsystem m_turret;
    private final RevTiltSubsystem m_tilt;

    public SimLimelight(RevTurretSubsystem turret, RevTiltSubsystem tilt) {
        m_turret = turret;
        m_tilt = tilt;

    }

    public boolean getIsTargetFound() {
        return Math.abs(getdegVerticalToTarget()) < 5 && Math.abs(getdegRotationToTarget()) < 5;
    }

    public double getdegRotationToTarget() {

        return m_turret.targetAngle - m_turret.getAngle();
    }

    /**
     * once inside .5 degrees, camera is considered on target
     * 
     * @return
     */
    public boolean getHorOnTarget() {
        return getIsTargetFound() && Math.abs(getdegRotationToTarget()) < .5;
    }

    /**
     * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     * 
     * @return
     */
    public double getdegVerticalToTarget() {

        return m_tilt.targetAngle - m_tilt.getAngle();
    }

    public boolean getVertOnTarget() {
        return getIsTargetFound() && Math.abs(getdegVerticalToTarget()) < .5;
    }

}
