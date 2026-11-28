package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.localization.LocalizationSubsystem;

/**
 * Turns the robot's pose into a shot: distance to the target, how far to aim, and the shooter RPM to use.
 * Pure math, no hardware, so it's easy to unit test and print to the Dashboard.
 *
 * Everything here is in FIELD coordinates (inches, radians, heading CCW from +X) -- the same frame as LocalizationSubsystem.
 */
public class ShotCalculator {

    // ---------------- FILL THESE IN ----------------
    // Center of the CELL opening for each end of YOUR alliance's hive, in field inches {x, y}.
    // The target is picked by HiveTargetSelector from which half of the field the robot is on:
    // the robot is assumed to shoot at the cell on its own side of SPLIT_Y.
    // Measure from the official field CAD, then verify with a tape measure on your practice field.
    // (If you ever play both alliances you'll need a mirrored pair for the other hive.)
    public static final double[] TARGET_LOW_Y = {36.9, -43.6};   // cell A (smaller X)
    public static final double[] TARGET_HIGH_Y = {66.7, -40.9};  // cell B (bigger X)

    // Where the shooter exit sits relative to the tracking point (robot frame): forward / left, inches.
    // If the turret pivot is off-center this matters a lot: distance must be measured from the shooter, not the robot center.
    public static final double SHOOTER_FWD_IN = 0.6;
    public static final double SHOOTER_LEFT_IN = 0;

    // Calibration table, sorted by distance: {distanceInches, flywheelRPM}.
    // The flywheel now runs all match, so this table is read every loop wherever the robot is. Outside the table's
    // range the value is clamped to the nearest end, so make the first and last rows sensible for the closest and
    // farthest places you'll drive, not just the places you'll shoot from.
    // PLACEHOLDERS ONLY (zeros). Fill from real test shots, then add more rows as needed.
    private static final double[][] TABLE = {
            {33, 3200},
            {41, 3360},
            {51.5, 3485},
            {52.5, 3550},
            {58.75, 3660},
            {65.5, 3600},
            {74, 3630},
            {83, 3950}
    };
    // -----------------------------------------------

    public static class Solution {
        public double distanceIn;        // shooter exit -> target
        public double bearingRad;        // field-frame direction from shooter to target
        public double relativeAngleRad;  // bearing minus robot heading, wrapped to (-pi, pi]. 0 = straight ahead, + = to the left
        public double rpm;
    }

    /** @param side which half of the field the robot is on, i.e. which cell is treated as the live target */
    public static Solution solve(double robotX, double robotY, double headingRad, HiveTargetSelector.Side side) {
        double c = Math.cos(headingRad);
        double s = Math.sin(headingRad);

        // shooter position in the field frame
        double shooterX = robotX + SHOOTER_FWD_IN * c - SHOOTER_LEFT_IN * s;
        double shooterY = robotY + SHOOTER_FWD_IN * s + SHOOTER_LEFT_IN * c;

        double[] target = (side == HiveTargetSelector.Side.HIGH_Y) ? TARGET_HIGH_Y : TARGET_LOW_Y;
        double dx = target[0] - shooterX;
        double dy = target[1] - shooterY;

        Solution sol = new Solution();
        sol.distanceIn = Math.hypot(dx, dy);
        sol.bearingRad = Math.atan2(dy, dx);
        sol.relativeAngleRad = LocalizationSubsystem.wrapRad(sol.bearingRad - headingRad);
        sol.rpm = interpolate(sol.distanceIn);
        return sol;
    }

    // linear interpolation, clamped to the ends of the table
    private static double interpolate(double distance) {
        if (distance <= TABLE[0][0]) return TABLE[0][1];
        for (int i = 1; i < TABLE.length; i++) {
            if (distance <= TABLE[i][0]) {
                double t = (distance - TABLE[i - 1][0]) / (TABLE[i][0] - TABLE[i - 1][0]);
                return TABLE[i - 1][1] + t * (TABLE[i][1] - TABLE[i - 1][1]);
            }
        }
        return TABLE[TABLE.length - 1][1];
    }
}