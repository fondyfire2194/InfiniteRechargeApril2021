// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class ShootData {

    /**
     * 
     */

    public static int crossLine = 0;
    public static int retractOneStraight = 1;

    public static int leftTwoBall = 2;
    public static int trench3Ball = 3;

    public static double lowShotMPS = 7.8;
    public static double lowShotTime = 4;

    public static double innerShotMPS = 8;
    public static double innerTiltAngle = 12;

    private static double shotHeight = FieldConstants.SHOT_HEIGHT;
    private static double innerWallFromOuter = .74;

    public static double activeTeleopShootSpeed;
    public static double activeTeleopTiltAngle;
    public static double activeTeleopTiltOffset;
    public static double activeTeleopTurretAngle;
    public static double activeTeleopTurretOffset;

    public static double frontOfRobotToCenterofTiltTurret = .25;// meters

    public ShootData() {
    }

    private static double getTiltAngle(double x) {
        return Math.toDegrees(Math.tan(shotHeight / x));
    }

    private static double getTurretAngleXY(double x, double y) {
        return Math.toDegrees(Math.atan(y / x));
    }

    private static double getFloorDistanceSqrd(double x, double y) {
        return ((x * x) + (y * y));
    }

    private static double getFloorDistance(double x, double y) {
        return Math.sqrt(getFloorDistanceSqrd(x, y));
    }

    private static double getShotDistance(double x, double y) {

        return Math.sqrt((getFloorDistanceSqrd(x, y)) + (shotHeight * shotHeight));
    }

    public final static class centerPowerPortConstants {
        public static double retractDistance = -1;
        public static double tiltAngle = 19.6;// getTiltAngle(4 + frontOfRobotToCenterofTiltTurret);// 22.7;//
        public static double turretAngle = 0;
        public static double shootSpeed = 30;
        public static double tiltOffset = 2;
        public static double turretOffset = 0;
        public static double shootTime = 4;

    }

    public final static class shieldGenConstants {
        private static double yfromPort = Units.inchesToMeters((74));
        public static double retractDistance = -4;
        public static double tiltAngle = 24;
        public static double turretAngle = 32;
        public static double shootSpeed = 38;/// ?
        public static double tiltOffset = 0;/// ?
        public static double turretOffset = 0;/// ?
        public static double shootTime = 5;

    }

    public final static class trenchShotConstants {
        public static double retractDistance = -4;
        public static double tiltAngle = 15;

        public static double turretAngle = -21;
        public static double shootSpeed = 35;
        public static double tiltOffset = 7;
        public static double turretOffset = 1;
        public static double shootTime = 5;

    }

    public final static class behindControlPanelShotConstants {
        
        public static double tiltAngle = 15;

        public static double turretAngle = -15;
        public static double shootSpeed = 40;
        public static double tiltOffset = 7;
        public static double turretOffset = 1;
        public static double shootTime = 5;

    }

    public final static class lowShotConstants {

        public static double tiltAngle = 29;
        public static double turretAngle = 0;
        public static double shootSpeed = 23;

    }

    public static void showValues() {
        SmartDashboard.putNumber("TAI",activeTeleopTiltAngle);
        SmartDashboard.putNumber("TAU",activeTeleopTurretAngle);
      

    }

}
