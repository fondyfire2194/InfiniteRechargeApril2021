// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ShootData {

    /**
     * array contents are pipeline, distance, shoot speed, shoot time, turret angle,
     * tilt angle.
     * 
     * 
     * 
     */

    private static double[][] shootValues =

            { { 0, 3., 4500, 5, -54, 4.7 }// front of power port at initiation line
                    , { 2, 3, 4, 5, 0 } //
                    , { 2, 3, 4, 5, 0 }, //
                    { 2, 3, 4, 5, 0 } //
            };

    public ShootData() {
    }

    public static int getPipeline(int pointer) {
        return (int) shootValues[pointer][0];
    }

    public static double getDistance(int pointer) {
        return shootValues[pointer][1];
    }

    public static double getShootSpeed(int pointer) {
        return shootValues[pointer][2];
    }

    public static double getShootTime(int pointer) {
        return shootValues[pointer][3];
    }

    public static double getTurretAngle(int pointer) {
        return shootValues[pointer][4];
    }

    public static double getTiltAngle(int pointer) {
        return shootValues[pointer][5];
    }

}