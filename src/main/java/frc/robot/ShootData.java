// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.FieldConstants;

/** Add your docs here. */
public class ShootData {

    /**
     * 
     * 
     * 
     * 
     */
    public static int crossLine = 0;
    public static int retractOneStraight = 1;

    public static int leftTwoBall = 2;
    public static int trenchTwoBall = 3;
    public static int trench3Ball = 4;

    private static final double shootTime = 5;
    private static final double xRetractDistance = -.75;
    private static final double xTrenchTwoBallPickup = -3;
    private static final double xLeftTwoBallPickup = -2;
    private static final double xTrenchThreeBallPickup = -4;

    private static final double allowInnerPort = -2.5;

    private static double yTrenchCenterFromPort = FieldMap.trenchCLtoGoalY;
    private static double yLeftPickupFromPort = Units.inchesToMeters(72.4);

    private static double initiationLine = FieldConstants.initiationLine;
    private static double shotHeight = FieldConstants.SHOT_HEIGHT;
    private static double innerWallFromOuter = .74;

    /**
     * 
     * array contents are
     * 
     * pipeline,
     * 
     * first moveDistance, second moveDistance, shootdistance
     * 
     * shoot time, turret angle, turret shift deg, tilt angle, tilt shift deg.
     * 
     * distances are absolute and need to be repeated if robot is to remain in place
     */

    private static double[][] shootValues = {

            // cross line 0 (not used but don't delete!
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 0 )

            { 0, xRetractDistance, xRetractDistance,
                    getShotDistance((initiationLine - xRetractDistance + innerWallFromOuter), 0), shootTime, 0, 0,
                    getOuterTiltAngle((initiationLine - xRetractDistance), 0), allowInnerPort },

            { 0, xLeftTwoBallPickup, xLeftTwoBallPickup,
                    getShotDistance((initiationLine - xLeftTwoBallPickup), yLeftPickupFromPort), shootTime,
                    getTurretAngleXY((initiationLine - xLeftTwoBallPickup), yLeftPickupFromPort), 0,
                    getOuterTiltAngle((initiationLine - xLeftTwoBallPickup), yLeftPickupFromPort), 0 },

            { 0, xTrenchTwoBallPickup, xTrenchTwoBallPickup,
                    getShotDistance((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), shootTime,
                    -getTurretAngleXY((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), 0,
                    getOuterTiltAngle((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), 0 },

            { 0, xRetractDistance, xTrenchThreeBallPickup,
                    getShotDistance((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), shootTime,
                    -getTurretAngleXY((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), 0,
                    getOuterTiltAngle((initiationLine - xTrenchTwoBallPickup), yTrenchCenterFromPort), 0 }

    };

    // private static double positionRate = 3;

    public ShootData() {
    }

    public static int getPipeline(int pointer) {
        return (int) shootValues[pointer][0];
    }

    public static double getFirstDistance(int pointer) {
        return shootValues[pointer][1];

    }

    public static double getSecondDistance(int pointer) {
        return shootValues[pointer][2];
    }

    public static double getShootDistance(int pointer) {
        return shootValues[pointer][3];
    }

    public static double getShootTime(int pointer) {
        return shootValues[pointer][4];
    }

    public static double getTurretAngle(int pointer) {
        return shootValues[pointer][5] + shootValues[pointer][6];
    }

    public static double getTurretOffset(int pointer) {
        return shootValues[pointer][6];
    }

    public static double getTiltAngle(int pointer) {
        return shootValues[pointer][7] + shootValues[pointer][8];
    }

    public static double getTiltOffset(int pointer) {
        return shootValues[pointer][8];
    }

    private static double getOuterTiltAngle(double x, double y) {
        return Math.toDegrees(Math.atan(shotHeight / getFloorDistance(x, y)));
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

    public static void showValues(int value) {

        SD.putN1("TiltAngle " + String.valueOf(value), getTiltAngle(value));
        SD.putN1("TurretAngle " + String.valueOf(value), getTurretAngle(value));
        SD.putN1("ShootDistance " + String.valueOf(value), getShootDistance(value));

    }

}
