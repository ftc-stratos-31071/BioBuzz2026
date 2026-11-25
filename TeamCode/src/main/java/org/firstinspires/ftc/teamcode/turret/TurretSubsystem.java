package org.firstinspires.ftc.teamcode.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret driven by two continuous-rotation servos and an absolute analog encoder.
 *
 * This version runs its own small PD loop on a NON-wrapping angle in (-180, 180]. The turret has a ~5 degree dead
 * zone at the back, so it must never take a "shortest path" across the 0/360 seam of the raw encoder: that path
 * can run straight through the gap and into the hard stop. Here the seam of the logical angle sits in the gap,
 * so the turret can only ever travel between its two stops.
 *
 * Angle convention: 0 = robot front, counter-clockwise positive (viewed from above).
 * Only standard FTC SDK classes are used (CRServo, AnalogInput).
 */
@Configurable
public class TurretSubsystem extends SubsystemBase {

    // raw encoder angle when the turret faces forward, increases CCW
    public static double FRONT_RAW_DEG = 0;
    // +1 if the raw angle increases as the turret rotates CCW
    public static double ENCODER_SIGN = -1;
    // +1 if positive servo power rotates the turret CCW
    public static double OUTPUT_SIGN = 1;
    // voltage the encoder reads at a full 360 degrees (your old code used 3.2)
    public static double ENCODER_MAX_VOLTS = 3.3;
    // both servos push the same gear; if they fight each other, flip one of these
    private static final boolean RIGHT_SERVO_REVERSED = false;
    private static final boolean LEFT_SERVO_REVERSED = false;

    // control gains (degrees in, power out). START LOW and tune from Panels.
    public static double kP = 0.0;
    public static double kD = 0.000;
    public static double kS = 0.0;            // small static-friction boost, if the turret stalls near the target
    public static double MAX_POWER = 0.3;     // keep low for first bring-up
    public static double DEADBAND_DEG = 0.5;
    public static boolean ENABLED = true;     // disables movement of turret, different from turning it off in the autoaimsubsystem, which allows for manual control instead of complete off-mode

    // you can type a value in Panels while auto-aim is off in the autoaimsubsystem, for bring-up
    // when AutoAimSubsystem controls the turret it overwrites this every loop.
    public static double targetAngleDeg = 0;

    private final CRServoEx servoR;
    private final CRServoEx servoL;
    private final AnalogInput encoder;
    private final TurretAimPlanner planner = new TurretAimPlanner();

    private static double rawAngleDeg = 0;
    private double measuredAngleDeg = 0;
    private double powerOut = 0;
    private double lastError = 0;
    private double lastTimeSec = 0;
    private boolean hasLast = false;
    private Telemetry telemetry;

    public TurretSubsystem(final HardwareMap hMap, final String servoNameR, final String servoNameL, final String encoderName) {
        servoR = new CRServoEx(hMap, servoNameR);
        servoL = new CRServoEx(hMap, servoNameL);
        encoder = hMap.get(AnalogInput.class, encoderName);

        servoR.setInverted(RIGHT_SERVO_REVERSED);
        servoL.setInverted(LEFT_SERVO_REVERSED);

        measuredAngleDeg = readAngleDeg();
        targetAngleDeg = measuredAngleDeg; // start by holding wherever we are
    }

    /**
     * aims at target given as an angle relative to the robot front (CCW+)
     * planner clamps it into a reachable range and handles dead zone at the back
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

    @Override
    public void periodic() {
        measuredAngleDeg = readAngleDeg();

        if (!ENABLED) { // hold its position if disabled
            setServoPower(0);
            powerOut = 0;
            hasLast = false;
            sendTelemetry();
            return;
        }

        // no wrapping on purpose, bc turret cannot cross the gap, so error is just target - measured.
        double error = targetAngleDeg - measuredAngleDeg;

        double now = System.nanoTime() / 1e9;
        double derivative = 0;
        if (hasLast) {
            double dt = Math.max(now - lastTimeSec, 1e-3);
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

    private double readAngleDeg() {
        rawAngleDeg = encoder.getVoltage() / ENCODER_MAX_VOLTS * 360.0;
//        rawAngleDeg=encoder.getVoltage();
        System.out.println(rawAngleDeg);
        return TurretAimPlanner.wrap180(ENCODER_SIGN * (rawAngleDeg - FRONT_RAW_DEG));
    }

    private void setServoPower(double p) {
        servoR.set(p);
        servoL.set(p);
    }

    private void sendTelemetry() {
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret raw angle", rawAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret angle", measuredAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret target", targetAngleDeg);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("turret power", powerOut);
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
}