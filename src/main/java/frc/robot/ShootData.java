// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ShootData {

    /**
     * 
     * 
     * 
     * 
     */

    private static double[][] shootValues =
            // array contents are pipeline, first moveDistance, second moveDistance,
            // shootdistance, shoot speed, shoot
            // time, turret angle, tilt angle.
            // front of power port at initiation line 0
            { { 0, 0, -1, 30, 4500, 5, 0, 74. }, // 0 center start shoot retract

                    { 0, -1, -1, 30, 3500, 5, 0, 63.5 }, // 1 center start retract shoot

                    { 0, 0, -1, 30, 3575, 5, 20, 63.5 }, // 2 left start shoot retract

                    { 0, -1, -1, 30, 4100, 5, 26, 64.5 }, // 3 left start retract shoot

                    { 0, 0, -1, 30, 3575, 5, -20, 63.5 }, // 4 right start shoot retract

                    { 0, -1, -1, 30, 4100, 5, -26, 64.5 }, // 5 right start retract shoot

                    { 0, -3, -3, 30, 4100, 2, -26, 61.5 }, // 6 trench start pickup 2 and shoot

                    { 0, -4, -3, 30, 4500, 5, -9, 63.5 } // 7 move to pick up 3rd trench ball and shoot
            };
    private static double positionRate = 3;

    public ShootData() {
    }

    public static double getPositionRate() {
        return positionRate;
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

    public static double getShootSpeed(int pointer) {
        return shootValues[pointer][4];
    }

    public static double getShootTime(int pointer) {
        return shootValues[pointer][5];
    }

    public static double getTurretAngle(int pointer) {
        return shootValues[pointer][6];
    }

    public static double getTiltAngle(int pointer) {
        return shootValues[pointer][7];
    }

}