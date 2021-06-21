/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimeLight;
import frc.robot.ShootData;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.SetTiltOffset;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.SetTurretOffset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.CellTransportSubsystem;
import frc.robot.subsystems.RearIntakeSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoModeShieldGenTestVision extends SequentialCommandGroup {
        /**
         * Creates a new Auto0.
         * 
         * Start in front of power port and shoot
         */
        static double retractDistance = ShootData.shieldGenConstants.retractDistance;
        static double tiltAngle = ShootData.shieldGenConstants.tiltAngle;
        static double turretAngle = ShootData.shieldGenConstants.turretAngle;
        static double shootSpeed = ShootData.shieldGenConstants.shootSpeed;
        static double tiltOffset = ShootData.shieldGenConstants.tiltOffset;
        static double turretOffset = ShootData.shieldGenConstants.turretOffset;
        static double shootTime = ShootData.shieldGenConstants.shootTime;

        static double retractDistance1 = ShootData.shieldGenConstants.retractDistance1;
        static double tiltAngle1 = ShootData.shieldGenConstants.tiltAngle1;
        static double turretAngle1 = ShootData.shieldGenConstants.turretAngle1;
        static double shootSpeed1 = ShootData.shieldGenConstants.shootSpeed1;
        static double tiltOffset1 = ShootData.shieldGenConstants.tiltOffset1;
        static double turretOffset1 = ShootData.shieldGenConstants.turretOffset1;
        static double shootTime1 = ShootData.shieldGenConstants.shootTime1;

        public AutoModeShieldGenTestVision(RevShooterSubsystem shooter, RevTurretSubsystem turret, RevTiltSubsystem tilt,
                        CellTransportSubsystem transport, RevDrivetrain drive, LimeLight limelight,
                        Compressor compressor, RearIntakeSubsystem intake) {
                // Add your commands in the super() call, e.g.
                // super(new FooCommand(), new BarCommand());
                //

                super(new ParallelCommandGroup(

                        new PositionTurret(turret, turretAngle), new PositionTilt(tilt, tiltAngle)),

                        new ParallelCommandGroup(new SetUpLimelightForTarget(limelight),
                                        new SetTiltOffset(tilt, tiltOffset),
                                        new SetTurretOffset(turret, turretOffset)));
}
}