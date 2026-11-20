package org.firstinspires.ftc.teamcode.shooter;

import android.provider.Settings;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.TelemetryData;
import com.seattlesolvers.solverslib.util.Timing;

@Configurable
public class ShooterSubsystem extends SubsystemBase {
    private final ServoEx servo;
    public final MotorEx motor1;
    public static double target_RPM = 3550;
    public static boolean motorRunning = false;
    public double servoAngle = 0;

    // PIDF variables for flywheel:
    @Sorter(sort = 0)
    public static double kP = 0.0004;
    @Sorter(sort = 1)
    public static double kI = 0;
    @Sorter(sort = 2)
    public static double kD = 0.0001;
    @Sorter(sort = 3)
    public static double kF = 0.000193;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private final Timing.Stopwatch timer;
    public ShooterSubsystem(final HardwareMap hMap, final String servoName, final String motorName1, Timing.Stopwatch timer) {
        servo = new ServoEx(hMap, servoName, 180);

        motor1 = new MotorEx(hMap, motorName1);
        motor1.setRunMode(MotorEx.RunMode.RawPower);
        motor1.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor1.setInverted(true);

        motor1.resetEncoder();

        this.timer = timer;
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
        return (motor1.getVelocity() * 60.0) / 28;
    }

    public void increaseRPM() {
        target_RPM += 100;
    }

    public void decreaseRPM() {
        target_RPM -= 100;
    }

    public void increaseHoodAngle() {
        // CREATE CONSTRAINTS FOR SERVO
        servoAngle += 10;
        servo.set(servoAngle/180);
    }

    public void decreaseHoodAngle() {
        servoAngle -= 10;
        servo.set(servoAngle/180);
    }
}