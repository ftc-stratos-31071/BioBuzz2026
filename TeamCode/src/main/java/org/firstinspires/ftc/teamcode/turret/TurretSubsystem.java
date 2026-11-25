package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

/**
 * Turret driven by two continuous-rotation servos and an absolute analog encoder that is geared to the turret.
 *
 * WHY THE ENCODER NEEDS SPECIAL HANDLING
 * The encoder is geared 103/36 (about 2.86) turns for every one turn of the turret. The analog encoder only reports
 * where it is inside ONE of its own turns (0..360 degrees, repeating), so a single raw reading could belong to two or
 * three different turret angles (they are 360/2.86 = about 126 degrees of turret motion apart). To get the real turret
 * angle we (1) count how many encoder turns have gone by, by "unwrapping" the reading, and (2) divide by the ratio.
 * All of that lives in readAngleDeg().
 *
 * ANGLE CONVENTION: 0 = robot front, counter-clockwise positive (viewed from above), same as the field heading.
 *
 * CONTROL: a small PD loop on that angle. The turret has a ~5 degree dead zone at the back and can never cross it, so
 * the error is simply target - measured, with no "shortest way around" wrapping.
 *
 * STARTUP REQUIREMENT: before every match the turret must be physically within about +/-60 degrees of the robot front
 * when the code starts (see readAngleDeg for why).
 */
@Configurable
public class TurretSubsystem extends SubsystemBase {

    // =====================================================================================================
    // ENCODER -> TURRET ANGLE SETTINGS (calibrate these; the tuning guide walks through each one)
    // =====================================================================================================

    // The raw encoder angle (0..360, shown as "turret raw angle" in Panels) when the turret points at the robot front.
    public static double FRONT_RAW_DEG = 28.3;
    // +1 if the raw encoder angle INCREASES when the turret rotates counter-clockwise (seen from above), -1 if it
    // DECREASES. Flip this if "turret angle" goes the wrong way when you rotate the turret by hand.
    public static double ENCODER_SIGN = 1;
    // The voltage the encoder reads at the very top of its range (just before it wraps back to 0). If this is wrong,
    // every time the encoder wraps the unwrapped angle jumps by a small error, and the error piles up turn after turn.
    public static double ENCODER_MAX_VOLTS = 3.3;
    // How many times the ENCODER turns for one full turn of the TURRET: encoder gear / turret gear teeth = 103 / 36.
    // Gear backlash and tooth counts mean the real value can differ slightly, so the tuning guide has you measure it.
    public static double ENCODER_TURNS_PER_TURRET_TURN = 103.0 / 36.0;
    // If more than this many seconds pass between two encoder reads (e.g. between INIT and START), we can't trust that
    // the turret didn't get moved by hand in the meantime, so we forget the turn count and re-do the startup guess.
    public static double GAP_RESET_SEC = 1.0;
    // The turret physically can't be past its limits. If the computed angle is more than this many degrees beyond them,
    // we assume the turn count is wrong and stop driving the turret (see angleValid).
    public static double RANGE_MARGIN_DEG = 5.0;

    // +1 if positive servo power rotates the turret counter-clockwise (seen from above), -1 if clockwise
    public static double OUTPUT_SIGN = 1;
    // both servos push the same gear; if they fight each other, flip one of these (they are code constants)
    private static final boolean RIGHT_SERVO_REVERSED = false;
    private static final boolean LEFT_SERVO_REVERSED = false;

    // =====================================================================================================
    // CONTROL GAINS (degrees in, servo power out). Start at 0 and raise them following the tuning guide.
    // =====================================================================================================
    public static double kP = 0.015;
    public static double kD = 0.000;
    public static double kS = 0.0;            // small static-friction boost, if the turret stalls just short of the target
    public static double MAX_POWER = 0.3;     // keep low for first bring-up
    public static double DEADBAND_DEG = 0.5;  // inside this error the turret stops driving, to avoid jitter
    // false = send 0 power (the turret stops being driven; it does NOT actively hold position). This is different from
    // turning off auto-aim in AutoAimSubsystem, which leaves the turret under manual/Panels control instead.
    public static boolean ENABLED = true;

    // Where the turret is trying to go. You can type a value here in Panels while auto-aim is off (bring-up).
    // When AutoAimSubsystem controls the turret it overwrites this every loop.
    public static double targetAngleDeg = 10;

    private final CRServoEx servoR;
    private final CRServoEx servoL;
    private final AnalogInput encoder;
    private final TurretAimPlanner planner = new TurretAimPlanner();

    // --- values kept for telemetry / control ---
    private static double rawAngleDeg = 0;     // encoder angle 0..360, straight from the voltage
    private double measuredAngleDeg = 0;       // the real turret angle, the output of readAngleDeg()
    private double powerOut = 0;
    private double lastError = 0;
    private double lastTimeSec = 0;
    private boolean hasLast = false;

    // --- state for the encoder unwrapping (everything readAngleDeg() needs to remember between calls) ---
    private double unwrappedEncDeg = 0;        // running encoder angle with the turn count included (can be < 0 or > 360)
    private double lastRawDeg = 0;             // previous raw reading, to measure how far the encoder moved
    private double lastReadSec = 0;            // when we last read, to detect long gaps
    private boolean haveReading = false;       // false until the first read has set up the turn count
    private boolean reinitialized = false;     // set when the turn count was (re)started, so periodic() can hold position
    private boolean angleValid = true;         // false if the computed angle is impossible (turn count probably wrong)

    public TurretSubsystem(final HardwareMap hMap, final String servoNameR, final String servoNameL, final String encoderName) {
        servoR = new CRServoEx(hMap, servoNameR);
        servoL = new CRServoEx(hMap, servoNameL);
        encoder = hMap.get(AnalogInput.class, encoderName);

        servoR.setInverted(RIGHT_SERVO_REVERSED);
        servoL.setInverted(LEFT_SERVO_REVERSED);

        measuredAngleDeg = readAngleDeg();   // first read: sets up the turn count (startup assumption)
        targetAngleDeg = measuredAngleDeg;   // start by holding wherever we are
    }

    /**
     * Aim at a target given as an angle relative to the robot front (CCW+). The planner clamps it into the reachable
     * range and handles the dead zone at the back.
     */
    public void setAimAngleDeg(double relativeTargetDeg) {
        targetAngleDeg = planner.plan(relativeTargetDeg);
    }

    public double getAngleDeg() {
        return measuredAngleDeg;
    }

    public double getTargetAngleDeg() {
        return targetAngleDeg;
    }

    public boolean isAimed(double toleranceDeg) {
        return Math.abs(targetAngleDeg - measuredAngleDeg) <= toleranceDeg;
    }

    /** false means the angle reading is impossible (the turn count is probably wrong) and the turret has been stopped. */
    public boolean isAngleValid() {
        return angleValid;
    }

    @Override
    public void periodic() {
        measuredAngleDeg = readAngleDeg();

        // readAngleDeg() just restarted the turn count (first loop, or a long gap such as INIT -> START). The turret may
        // have been moved by hand since the target was set, so hold wherever it is now instead of driving back to an
        // old target, and clear the derivative history so we don't get a one-loop spike.
        if (reinitialized) {
            targetAngleDeg = measuredAngleDeg;
            hasLast = false;
            reinitialized = false;
        }

        // stop driving if disabled, or if the angle is impossible (a wrong angle could drive the turret into a hard stop)
        if (!ENABLED || !angleValid) {
            setServoPower(0);
            powerOut = 0;
            hasLast = false;
            sendTelemetry();
            return;
        }

        // no wrapping on purpose: the turret can't cross the gap, so the error is just target - measured
        double error = targetAngleDeg - measuredAngleDeg;

        double now = System.nanoTime() / 1e9;
        double derivative = 0;
        if (hasLast) {
            double dt = Math.max(now - lastTimeSec, 1e-3); // never divide by ~0
            derivative = (error - lastError) / dt;
        }
        lastError = error;
        lastTimeSec = now;
        hasLast = true;

        double u = 0;
        if (Math.abs(error) > DEADBAND_DEG) {
            u = kP * error + kD * derivative + kS * Math.signum(error);
        }
        u = Math.max(-MAX_POWER, Math.min(MAX_POWER, u));

        // never push further into a limit
        if (measuredAngleDeg >= TurretAimPlanner.MAX_LIMIT_DEG && u > 0) u = 0;
        if (measuredAngleDeg <= TurretAimPlanner.MIN_LIMIT_DEG && u < 0) u = 0;

        powerOut = u;
        setServoPower(u * OUTPUT_SIGN);
        sendTelemetry();
    }

    /**
     * Converts the encoder voltage into the REAL turret angle in degrees (0 = front, CCW+).
     *
     * Call it once per loop (periodic() does). All the geared-encoder logic is in here:
     *
     *   1. voltage -> raw encoder angle, 0..360.
     *   2. Work out the "unwrapped" encoder angle, i.e. the raw angle PLUS the number of full turns the encoder has made:
     *        - First call (or after a long gap): we don't know the turn count. Assume the turret is within about
     *          +/-60 degrees of the front, which means the encoder is within +/-180 degrees of its front reading, so
     *          the nearest-to-front turn is the right one. (Only the TURN is guessed; the fine angle inside that turn
     *          still comes from the real reading, so the result is accurate, not zeroed.)
     *        - Every other call: see how far the raw angle moved since last time. Since the encoder can't move more
     *          than half a turn between loops, wrap that movement into -180..+180 and add it. A jump from 359 to 1
     *          degrees becomes +2, not -358, which is exactly how the turn count is tracked. Adding up the moves never
     *          drifts, because the small noise in each reading cancels out when you add them up.
     *   3. Turn encoder degrees into turret degrees: subtract the front reading, divide by the gear ratio, apply the sign.
     *   4. Sanity check: the turret can't really be beyond its limits, so an impossible angle means the turn count is
     *      wrong. angleValid is set, and periodic() stops driving the turret.
     */
    private double readAngleDeg() {
        // 1. voltage -> raw encoder angle
        rawAngleDeg = encoder.getVoltage() / ENCODER_MAX_VOLTS * 360.0;
        double now = System.nanoTime() / 1e9;

        // 2. unwrap
        if (!haveReading || now - lastReadSec > GAP_RESET_SEC) {
            // startup guess: take the turn that puts the turret closest to the front
            unwrappedEncDeg = FRONT_RAW_DEG + TurretAimPlanner.wrap180(rawAngleDeg - FRONT_RAW_DEG);
            haveReading = true;
            reinitialized = true;
        } else {
            // normal tracking: add how far the encoder moved since the last read (wrapped into -180..180)
            unwrappedEncDeg += TurretAimPlanner.wrap180(rawAngleDeg - lastRawDeg);
        }
        lastRawDeg = rawAngleDeg;
        lastReadSec = now;

        // 3. encoder degrees -> turret degrees
        double turretDeg = ENCODER_SIGN * (unwrappedEncDeg - FRONT_RAW_DEG) / ENCODER_TURNS_PER_TURRET_TURN;

        // 4. sanity check against the turret's physical range
        angleValid = turretDeg >= TurretAimPlanner.MIN_LIMIT_DEG - RANGE_MARGIN_DEG
                && turretDeg <= TurretAimPlanner.MAX_LIMIT_DEG + RANGE_MARGIN_DEG;

        return turretDeg;
    }

    private void setServoPower(double p) {
        servoR.set(p);
        servoL.set(p);
    }

    private void sendTelemetry() {
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret raw angle", rawAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret encoder unwrapped", unwrappedEncDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret angle", measuredAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret target", targetAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret power", powerOut);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret angle valid (1/0)", angleValid ? 1 : 0);
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
}