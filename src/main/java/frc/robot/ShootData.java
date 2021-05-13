// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class ShootData {

    /**
     * 
     * 
     * 
     * 
     */
    private static final double shootTime = 5;
    private static final double retractDistance = -1;
    private static final double twoBallPickup = -3;

    private static double[][] shootValues =

            /**
             * 
             * array contents are
             * 
             * pipeline,
             * 
             * first moveDistance, second moveDistance,shootdistance
             * 
             * shoot time, turret angle, tilt angle.
             * 
             * distances are absolute and need to be repeated if robot is to remain in place
             */
            // front of power port at initiation line 0
            {

                    { 0, 0, retractDistance, 3.5, shootTime, 0, 30.3 }, // 0 center start shoot retract

                    { 0, retractDistance, retractDistance, 4.4, shootTime, 0, 23.7 }, // 1 center start retract
                                                                                      // shoot

                    { 0, 0, retractDistance, 3.7, shootTime, 19.6, 28.8 }, // 2 left start shoot retract

                    { 0, retractDistance, retractDistance, 4.6, shootTime, 15, 23 }, // 3 left start retract
                                                                                     // shoot

                    { 0, 0, retractDistance, 3.7, shootTime, -19.6, 28.8 }, // 4 right start shoot retract

                    { 0, retractDistance, retractDistance, 4.6, shootTime, -15, 23 }, // 5 right start retract
                                                                                      // shoot

                    { 0, twoBallPickup, twoBallPickup, 5.5, shootTime, -26, 19 }, // 6 trench start pickup 2 and shoot

                    { 0, twoBallPickup - 1, twoBallPickup, 5.5, shootTime, -26, 19 } // 7 move to pick up 3rd
                                                                                     // trench ball and shoot

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
        return shootValues[pointer][5];
    }

    public static double getTiltAngle(int pointer) {
        return shootValues[pointer][6];
    }

}