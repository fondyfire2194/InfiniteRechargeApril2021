package frc.robot;

import java.util.ArrayList;

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

        public static final double leftStartY = targetCenterPointY + robotWidth + .2;

        public static final double rightStartY = targetCenterPointY - (robotWidth + .2);
        // public static ArrayList<Translation2d> ballPositions = new
        // ArrayList<Translation2d>();
        public static Translation2d[] ballPosition = new Translation2d[12];
        public static Pose2d[] startPosition = new Pose2d[5];

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
        public static double goalHeight = Units.inchesToMeters(96);
        public static double goalHeightSquared = Units.inchesToMeters(96 * 96);
        public static double shooterHeight = Units.inchesToMeters(26);
        public static double heightToShoot = goalHeight - shooterHeight;
        public static double heightToShootSquared = heightToShoot * heightToShoot;
        public static double trenchCLtoGoalY = targetCenterPointY - ourTrenchY;

        // Floor distances are neede to compute tilt angles and shot distances

        public static double leftStartFloorDistance = Math
                        .sqrt(initiationLine * initiationLine + ((robotWidth + .2) * (robotWidth + .2)));

        public static double rightStartFloorDistance = leftStartFloorDistance;

        public static double centerRetractFloorDistance = initiationLine + 1;

        public static final double leftRetractFloorDistance = Math
                        .sqrt((initiationLine + 1) * (initiationLine + 1) + ((robotWidth + .2) * (robotWidth + .2)));

        public static final double rightRetractFloorDistance = leftRetractFloorDistance;

        public static double frontTrenchFloorDistance = Math
                        .sqrt(((trenchCLtoGoalY * trenchCLtoGoalY) + (initiationLine + firstCellFromInitiationLine)
                                        * (initiationLine + firstCellFromInitiationLine)));

        public static double midTrenchFloorDistance = Math.sqrt(((trenchCLtoGoalY * trenchCLtoGoalY)
                        + ((initiationLine + firstCellFromInitiationLine) + distanceBetweenTrenchCells)
                                        * ((initiationLine + firstCellFromInitiationLine)
                                                        + distanceBetweenTrenchCells)));

        public static double rearTrenchFloorDistance = Math.sqrt(((trenchCLtoGoalY * trenchCLtoGoalY) + (initiationLine
                        + firstCellFromInitiationLine + 2 * distanceBetweenTrenchCells)
                        * (initiationLine + firstCellFromInitiationLine + 2 * distanceBetweenTrenchCells)));

        public static double oppTrenchFloorDistance = Math.sqrt(
                        ((trenchCLtoGoalY * trenchCLtoGoalY) + ((fieldLength / 2) + 2) * ((fieldLength / 2) + 2)));

        // Shot distances

        public static double initLineCenterShotDistance = Math
                        .sqrt(heightToShootSquared + initiationLine * initiationLine);

        public static double initLineLeftShotDistance = Math
                        .sqrt(heightToShootSquared + leftStartFloorDistance * leftStartFloorDistance);

        public static double initLineRightShotDistance = initLineLeftShotDistance;

        public static double centerRetractShotDistance = Math
                        .sqrt(heightToShootSquared + (centerRetractFloorDistance * centerRetractFloorDistance));

        public static double leftRetractShotDistance = Math
                        .sqrt(heightToShootSquared + (leftRetractFloorDistance * leftRetractFloorDistance));

        public static double rightRetractShotDistance = leftRetractShotDistance;

        public static double frontTrenchShotDistance = Math
                        .sqrt(heightToShootSquared + frontTrenchFloorDistance * frontTrenchFloorDistance);

        public static double midTrenchShotDistance = Math
                        .sqrt(heightToShootSquared + midTrenchFloorDistance * midTrenchFloorDistance);

        public static double rearTrenchShotDistance = Math
                        .sqrt(heightToShootSquared + rearTrenchFloorDistance * rearTrenchFloorDistance);

        public static double oppTrenchShotDistance = Math
                        .sqrt(heightToShootSquared + ((fieldLength / 2) + 2) * ((fieldLength / 2) + 2));

        // Turret angles

        public static final double centerStartTurretAngle = 0;

        public static final double leftStartTurretAngle = Math.toDegrees(Math.atan((robotWidth + .2) / initiationLine));

        public static final double rightStartTurretAngle = -leftStartTurretAngle;

        public static final double retractTurretAngle = 0;

        public static final double retractLeftTurretAngle = Math
                        .toDegrees(Math.atan((robotWidth + .2) / (initiationLine + 1)));

        public static final double retractRightTurretAngle = -retractLeftTurretAngle;

        public static double frontTrenchTurretAngle = -Math
                        .toDegrees(Math.atan(targetCenterPointY / (initiationLine + firstCellFromInitiationLine)));

        public static double midTrenchTurretAngle = -Math.toDegrees(Math.atan(targetCenterPointY
                        / (initiationLine + firstCellFromInitiationLine + distanceBetweenTrenchCells)));

        public static double rearTrenchTurretAngle = -Math.toDegrees(Math.atan(targetCenterPointY
                        / (initiationLine + firstCellFromInitiationLine + distanceBetweenTrenchCells * 2))); // Tilt
                                                                                                             // angles

        public static double oppTrenchTurretAngle = -Math
                        .toDegrees(Math.atan(targetCenterPointY / ((fieldLength / 2) + 2)));

        // Tilt angles

        public static double centerStartTiltAngle = Math.toDegrees(Math.atan(heightToShoot / initiationLine));

        public static double leftStartTiltAngle = Math.toDegrees(Math.atan((heightToShoot) / leftStartFloorDistance));

        public static double rightStartTiltAngle = leftStartTiltAngle;

        public static double centerRetractTiltAngle = Math.toDegrees(Math.atan(heightToShoot / (initiationLine + 1)));

        public static double leftRetractTiltAngle = Math
                        .toDegrees(Math.atan((heightToShoot) / leftRetractFloorDistance));

        public static double rightRetractTiltAngle = leftRetractTiltAngle;

        public static double frontTrenchTiltAngle = Math
                        .toDegrees(Math.atan((heightToShoot) / frontTrenchFloorDistance));

        public static double midTrenchTiltAngle = Math.toDegrees(Math.atan((heightToShoot) / midTrenchFloorDistance));

        public static double rearTrenchTiltAngle = Math.toDegrees(Math.atan((heightToShoot) / rearTrenchFloorDistance));

        public static double oppTrenchTiltAngle = Math.toDegrees(Math.atan((heightToShoot) / oppTrenchFloorDistance));

        static {

                // start positions are in terms of the robot center (x,y)

                // lined up with target center
                startPosition[0] = new Pose2d(startPositionX, targetCenterPointY, new Rotation2d(0.0));
                // 40 inches closer to center of field LeftStart
                startPosition[1] = new Pose2d(startPositionX, targetCenterPointY + leftStartY, new Rotation2d(0.0));
                // 40 inches closer to wall (relative to target center) Right Start
                startPosition[2] = new Pose2d(startPositionX, targetCenterPointY - rightStartY, new Rotation2d(0.0));
                // Lined up with center line of trench
                startPosition[3] = new Pose2d(startPositionX, ourTrenchY + Units.inchesToMeters(rW2),
                                new Rotation2d(0.0));
                // centered in front of other team station (cross line)
                startPosition[4] = new Pose2d(startPositionX, fieldWidth - Units.inchesToMeters(94.66),
                                new Rotation2d(0.0));

        };

        public static void showValues() {
               // SD.putN1("Ht2Sh", heightToShoot);

                SD.putN1("FLDCS", initiationLine);
                SD.putN1("FLDLS", leftStartFloorDistance);
                SD.putN1("FLDRS", rightStartFloorDistance);
                SD.putN1("FLDCSR", initiationLine + 1);
                SD.putN1("FLDLSR", leftRetractFloorDistance);
                SD.putN1("FLDRSR", rightRetractFloorDistance);

                SD.putN1("FLDFT", frontTrenchFloorDistance);
                SD.putN1("FLDMT", midTrenchFloorDistance);
                SD.putN1("FLDRT", rearTrenchFloorDistance);
                SD.putN1("FLDOT", oppTrenchFloorDistance);

                SD.putN1("SHDCS", initLineCenterShotDistance);
                SD.putN1("SHDLS", initLineLeftShotDistance);
                SD.putN1("SHDRS", initLineRightShotDistance);
                SD.putN1("SHDCSR", centerRetractShotDistance);
                SD.putN1("SHDLSR", leftRetractShotDistance);
                SD.putN1("SHDRSR", rightRetractShotDistance);

                SD.putN1("SHDFT", frontTrenchShotDistance);
                SD.putN1("SHDMT", midTrenchShotDistance);
                SD.putN1("SHDRT", rearTrenchShotDistance);
                SD.putN1("SHDOT", oppTrenchShotDistance);

                SD.putN1("TUACS", centerStartTurretAngle);
                SD.putN1("TUALS", leftStartTurretAngle);
                SD.putN1("TUARS", rightStartTurretAngle);
                SD.putN1("TUACSR", retractTurretAngle);
                SD.putN1("TUALSR", retractLeftTurretAngle);
                SD.putN1("TUARSR", retractRightTurretAngle);

                SD.putN1("TUAFT", frontTrenchTurretAngle);
                SD.putN1("TUAMT", midTrenchTurretAngle);
                SD.putN1("TUART", rearTrenchTurretAngle);
                SD.putN1("TUAOT", oppTrenchTurretAngle);



                SD.putN1("TIACS", centerStartTiltAngle);
                SD.putN1("TIALS", leftStartTiltAngle);
                SD.putN1("TIARS", rightStartTiltAngle);
                SD.putN1("TIACSR", centerRetractTiltAngle);
                SD.putN1("TIALSR", leftRetractTiltAngle);
                SD.putN1("TIARSR", rightRetractTiltAngle);

                SD.putN1("TIAFT", frontTrenchTiltAngle);
                SD.putN1("TIAMT", midTrenchTiltAngle);
                SD.putN1("TIART", rearTrenchTiltAngle);
                SD.putN1("TIAOT", oppTrenchTiltAngle);

        }

}
