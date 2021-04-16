// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ShootData {

    /**
     * array contents are distance, shoot speed, turret angle, tilt angle and
     * pipeline
     * 
     * 
     * 
     */

    private static double[][] shootValues =

            { { 2, 3, 4, 5, 6 }// front of power port at initiation line
                    , { 2, 3, 4, 5, 6 } //
                    , { 2, 3, 4, 5, 6 }, //
                    { 2, 3, 4, 5, 6 } //
            };

    public ShootData() {
    }

    public static double getDistance(int pointer) {
        return shootValues[pointer][0];
    }

    public static double getShootSpeed(int pointer) {
        return shootValues[pointer][1];
    }

    public static double getTurretAngle(int pointer) {
        return shootValues[pointer][2];
    }

    public static double getTiltAngle(int pointer) {
        return shootValues[pointer][3];
    }

    public static int getPipeline(int pointer) {
        return (int) shootValues[pointer][4];
    }

}