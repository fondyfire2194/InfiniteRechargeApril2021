/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands.PowerPort;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.Shooter.WaitTiltTurretLocked;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterPowerPortToTargetOnly extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port, retract and shoot
         */
        static double tiltAngle = ShootData.centerPowerPortConstants.tiltAngle;
        static double turretAngle = ShootData.centerPowerPortConstants.turretAngle;

        static double tiltOffset = ShootData.centerPowerPortConstants.tiltOffset;
        static double turretOffset = ShootData.centerPowerPortConstants.turretOffset;

        public CenterPowerPortToTargetOnly(RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        RevShooterSubsystem shooter, LimeLight limelight) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());

                super(

                                new SetUpLimelightForTarget(limelight, limelight.activeStraightPipeline, false),

                                new ParallelCommandGroup(new SetTiltOffset(tilt, tiltOffset),
                                                new SetTurretOffset(turret, turretOffset),

                                                new PositionTilt(tilt, tiltAngle + tiltOffset),
                                                new PositionTurret(turret, turretAngle + turretOffset)),

                                new UseVision(limelight, true),
                                
                                new WaitTiltTurretLocked(tilt, turret).deadlineWith(
                                                new PositionHoldTilt(tilt, shooter, limelight),
                                                new PositionHoldTurret(turret, shooter, limelight)));

        }
}
