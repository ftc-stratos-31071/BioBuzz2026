package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Handles the fact that the turret can't spin all the way around.
 *
 * Turret angle convention used everywhere: 0 = robot front, counter-clockwise positive (viewed from above),
 * and the unreachable gap sits at the back, around +/-180 degrees. With ~355 degrees of travel the reachable
 * range is about -177.5 .. +177.5.
 *
 * Because the turret can never cross the gap, the "shortest way around" does not exist near the back: going from
 * +170 to -170 means swinging nearly the whole 355 degrees through the front. This class
 *   1. clamps requests into the reachable range (with a small safety margin so we never press on the hard stop), and
 *   2. adds hysteresis at the back so a target jittering across directly-behind-the-robot doesn't whip the turret
 *      around the long way for a few degrees of gain.
 *
 * No hardware in here, so it can be tested on its own.
 */
@Configurable
public class TurretAimPlanner {

    // Reachable limits in turret degrees. Defaults assume a 355 degree range with the gap centered at the back.
    // After measuring where your hard stops actually read, put those numbers here.
    public static double MIN_LIMIT_DEG = -177.5;
    public static double MAX_LIMIT_DEG = 177.5;

    // never command closer than this to a hard stop
    public static double SAFETY_MARGIN_DEG = 1.0;

    // If the target is within this many degrees of directly behind the robot and moves to the other side of the gap,
    // hold at the stop instead of swinging around. Worst-case aim error while holding is about
    // SEAM_HOLD_DEG + 2.5 degrees. Set to 0 to always take the long swing immediately.
    public static double SEAM_HOLD_DEG = 10.0;

    private double lastCommand = 0;

    /**
     * @param desiredDeg robot-relative angle to the target (0 = front, CCW+). Any angle; it is wrapped first.
     * @return the turret angle to command, always inside the reachable range
     */
    public double plan(double desiredDeg) {
        double d = wrap180(desiredDeg);
        double lo = MIN_LIMIT_DEG + SAFETY_MARGIN_DEG;
        double hi = MAX_LIMIT_DEG - SAFETY_MARGIN_DEG;

        double rearBand = 180.0 - SEAM_HOLD_DEG;
        boolean targetNearRear = Math.abs(d) > rearBand;
        boolean turretNearRear = Math.abs(lastCommand) > rearBand;

        if (targetNearRear && turretNearRear && Math.signum(d) != Math.signum(lastCommand)) {
            // the target crossed behind us: re-express it on the side we're already on, so the clamp below
            // pins us at that side's stop instead of sending the turret all the way around
            d += (lastCommand > 0) ? 360.0 : -360.0;
        }

        double cmd = Math.max(lo, Math.min(hi, d));
        lastCommand = cmd;
        return cmd;
    }

    /** Forget the previous command (e.g. after the turret has been re-homed). */
    public void reset() {
        lastCommand = 0;
    }

    /** Wrap to (-180, 180]. */
    public static double wrap180(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
