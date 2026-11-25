package org.firstinspires.ftc.teamcode.shooter;

import android.provider.Settings;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.seattlesolvers.solverslib.util.Timing;

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    public final MotorEx motor1;
    public static double target_RPM = 3550;
    public static boolean motorRunning = false;

    // PIDF variables for flywheel:
    @Sorter(sort = 0)
    public static double kP = 0.0004;
    @Sorter(sort = 1)
    public static double kI = 0;
    @Sorter(sort = 2)
    public static double kD = 0.0001;
    @Sorter(sort = 3)
    public static double kF = 0.000193;
    public static double NOMINAL_VOLTAGE = 12.5;
    private final VoltageSensor voltageSensor;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private final Timing.Stopwatch timer;
    public ShooterSubsystem(final HardwareMap hMap, final String motorName1, Timing.Stopwatch timer) {
        motor1 = new MotorEx(hMap, motorName1);
        motor1.setRunMode(MotorEx.RunMode.RawPower);
        motor1.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor1.setInverted(true);

        motor1.resetEncoder();
        rpmTrim = 0; // statics survive between OpMode runs, so start each run with no trim

        this.timer = timer;

        voltageSensor = hMap.voltageSensor.iterator().next();
    }

    @Override
    public void periodic() {
        if (motorRunning) {
            timer.resume(); // in case it was paused
            // PIDF STUFF:
            double error = target_RPM - getMeasuredRPM();

            double elapsed = timer.elapsedTime();
            double deltaTime = elapsed - lastTime;
            if (deltaTime == 0) deltaTime = 0.0001; // just in case so that we don't get undefined results if two loops run on same millisecond and therefore delta time = 0
            lastTime = elapsed;

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double output = kF * target_RPM + kP * error + kI * integral + kD * derivative;
            output *= NOMINAL_VOLTAGE / voltageSensor.getVoltage(); // voltage compensation

            motor1.set(output);
        } else {
            motor1.stopMotor();
            timer.pause();
        }

        PanelsTelemetry.INSTANCE.getTelemetry().addData("target rpm", target_RPM);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("measured rpm", getMeasuredRPM());
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public void toggleMotor() {
        // motor measures in ticks per second, not rpm - the motor has 28 ticks per revolution
        motorRunning = !motorRunning;
    }

    // next 3 methods are for telemetry output:
    public boolean isRunning() {
        return motorRunning;
    }

    public double getTargetRPM() {
        return motorRunning ? target_RPM : 0;
    }

    public double getMeasuredRPM() {
        return (motor1.getVelocity() * 60) / 28;
    }

    // ---- RPM: AutoAimSubsystem sets the base value every loop; the D-pad commands only nudge a trim on top of it ----
    public static double rpmTrim = 0;
    private double autoRPM = target_RPM; // last value from auto-aim (or the starting RPM before auto-aim runs)

    public void setTargetRPM(double rpm) {
        autoRPM = rpm;
        target_RPM = autoRPM + rpmTrim;
    }

    public void increaseRPM() {
        rpmTrim += 100;
        target_RPM = autoRPM + rpmTrim;
    }

    public void decreaseRPM() {
        rpmTrim -= 100;
        target_RPM = autoRPM + rpmTrim;
    }
}