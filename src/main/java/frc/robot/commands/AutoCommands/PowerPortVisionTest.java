/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PowerPortVisionTest extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */
        static double retractDistance = ShootData.centerPowerPortConstants.retractDistance;
        static double tiltAngle = ShootData.centerPowerPortConstants.tiltAngle;
        static double turretAngle = ShootData.centerPowerPortConstants.turretAngle;
        static double shootSpeed = ShootData.centerPowerPortConstants.shootSpeed;
        static double tiltOffset = ShootData.centerPowerPortConstants.tiltOffset;
        static double turretOffset = ShootData.centerPowerPortConstants.turretOffset;
        static double shootTime = ShootData.centerPowerPortConstants.shootTime;

        public PowerPortVisionTest(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(new ParallelCommandGroup(

                                new PositionTurret(turret, turretAngle), new PositionTilt(tilt, tiltAngle)),

                                new ParallelCommandGroup(new SetUpLimelightForTarget(limelight),
                                                new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset)));
        }
}