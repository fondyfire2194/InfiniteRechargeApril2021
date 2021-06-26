package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;

public class FieldMap {

        // Field marks, for reference
        public static final double fieldWidth = Units.inchesToMeters(323);
        public static final double fieldLength = Units.inchesToMeters(629.25);
        public static final double initiationLine = Units.inchesToMeters(120);

        // Distance to the center of the white tape line

        public static final double distanceBetweenTrenchCells = Units.inchesToMeters(36);
        public static final double firstCellFromInitiationLine = Units.inchesToMeters(36);

        // Robot dimensions with bumpers
        public static final double robotWidth = Units.inchesToMeters(34.75);
        public static final double robotLength = Units.inchesToMeters(37.75);

        // front of robot on line
        public static final double startLineX = fieldLength - initiationLine + robotLength;
        // half dimensions - for getting the center point of the robot
        public static final double rW2 = robotWidth / 2.0;
        public static final double rL2 = robotLength / 2.0;

        // centerline is the robot starting position
        public static final double targetCenterPointY = Units.inchesToMeters(94.66);
        // Y position of the goal
        public static final Translation2d goalCenterPoint = new Translation2d(fieldLength, targetCenterPointY);
        // center the robot on the startpoint (Outside of initiation line just
        // overlapping)
        public static final double startPositionX = startLineX - 2 - 2 * Constants.inchToMetersConversionFactor;
        // center line through long axis of trench
        public static final double ourTrenchY = Units.inchesToMeters(27.75);

        public static final double leftStartY = targetCenterPointY + Units.inchesToMeters(24) + robotWidth + .2;

        public static final double rightStartY = targetCenterPointY - (robotWidth + .2);
        // public static ArrayList<Translation2d> ballPositions = new
        // ArrayList<Translation2d>();
        public static Translation2d[] ballPosition = new Translation2d[12];
        public static Pose2d[] startPosition = new Pose2d[6];

        /**
         * Equations for tilt angle need to calculate diagonal floor distance then use
         * that as the adjacent in the tan equation
         * 
         * So diagonal floor distance = square root(distance from goal center squared
         * plus initition line squared
         * 
         * Tilt angle is then tan-1(floor distance / goalheight)
         * 
         */

        public static double trenchCLtoGoalY = targetCenterPointY - ourTrenchY;

        static {

                // start positions are in terms of the robot front x and robot mid Y

                // centered in front of other team station (cross line)
                startPosition[0] = new Pose2d(startPositionX, fieldWidth - Units.inchesToMeters(94.66),
                                new Rotation2d(0.0));

                // lined up with target center
                startPosition[1] = new Pose2d(startPositionX, targetCenterPointY, new Rotation2d(0.0));

                startPosition[2] = new Pose2d(startPositionX, leftStartY, new Rotation2d(0.0));

                startPosition[3] = new Pose2d(startPositionX, ourTrenchY + robotWidth / 2, new Rotation2d(0.0));

                startPosition[4] = new Pose2d(startPositionX, targetCenterPointY - robotWidth,
                                new Rotation2d(0.0));
        };

}
