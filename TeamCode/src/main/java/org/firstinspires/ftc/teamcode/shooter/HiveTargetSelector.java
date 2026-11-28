package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Decides which cell of the hive is the live target, using only which half of the field the robot is in.
 * (Assumes the driver never shoots unless the robot is on the same side as the upward-facing cell.)
 *
 * The shooter runs for the whole match, so there is no "shoot press" moment to decide at: the side simply
 * follows the robot's y every loop. A small dead band around SPLIT_Y stops the turret from flip-flopping between
 * the two cells when the robot is parked near the midline.
 *
 * No hardware in here, so it can be tested on its own.
 */
@Configurable
public class HiveTargetSelector {

    public enum Side { LOW_Y, HIGH_Y }

    // ---------------- FILL THIS IN ----------------
    // The y value (field inches, in YOUR field frame) that splits the field into the two halves.
    // Your ResetPositionCommand zeroes the pose at (0, 0), so this is the y of the middle of the field
    // measured from wherever you zero (0 only if you zero at the field's center line).
    public static double SPLIT_Y = 51.8; // halfway between the two cells (in X)
    // The side only flips once the robot is this far past SPLIT_Y (total band width, inches). 0 = flip exactly at SPLIT_Y.
    public static double HYSTERESIS_IN = 4;
    // -----------------------------------------------

    private Side side = Side.LOW_Y;
    private boolean initialized = false;

    /** Call every loop with the robot's y. */
    public void update(double robotY) {
        if (!initialized) {
            side = robotY >= SPLIT_Y ? Side.HIGH_Y : Side.LOW_Y;
            initialized = true;
            return;
        }
        double half = HYSTERESIS_IN / 2.0;
        if (side == Side.LOW_Y && robotY > SPLIT_Y + half) {
            side = Side.HIGH_Y;
        } else if (side == Side.HIGH_Y && robotY < SPLIT_Y - half) {
            side = Side.LOW_Y;
        }
    }

    public Side getSide() {
        return side;
    }
}
