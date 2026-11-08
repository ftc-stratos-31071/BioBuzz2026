package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;

@Config
public class ShooterSubsystem extends SubsystemBase {
    // temp vars:
    public static double rawPower = 0;

    private final ServoEx servo;
    public final MotorEx motor1;
    public final MotorEx motor2;
    public static double RPM = 500;
    public static boolean motorRunning = false;
    public static double servoAngle = 0;
    public static double measuredRPM;

    // PIDF variables for flywheel:
    public static double a_P = 0.002;
    public static double b_I = 0;
    public static double c_D = 0.01;
    public static double d_F = 0.0003;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private final Timing.Stopwatch timer;
    public ShooterSubsystem(final HardwareMap hMap, final String servoName, final String motorName1, final String motorName2, Timing.Stopwatch timer) {
        servo = new ServoEx(hMap, servoName, 180);

        motor1 = new MotorEx(hMap, motorName1);
        motor1.setRunMode(MotorEx.RunMode.RawPower);
        motor1.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motor2 = new MotorEx(hMap, motorName2);
        motor2.setRunMode(MotorEx.RunMode.RawPower);
        motor2.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motor1.resetEncoder();

        motor2.resetEncoder();

        this.timer = timer;
    }

    @Override
    public void periodic() {
        if (motorRunning) {
            timer.resume(); // in case it was paused
            // PIDF STUFF:
            double error = RPM - getMeasuredRPM();

            double elapsed = timer.elapsedTime();
            double deltaTime = elapsed - lastTime;
            if (deltaTime == 0) deltaTime = 0.0001; // just in case so that we don't get undefined results if two loops run on same millisecond and therefore delta time = 0
            lastTime = elapsed;

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double output = d_F * RPM + a_P * error + b_I * integral + c_D * derivative;

            motor1.set(output);
            motor2.set(output);
        } else {
            motor1.stopMotor();
            motor2.stopMotor();
            timer.pause();
        }

        measuredRPM = getMeasuredRPM(); // so i can see current rpm in ftc dashboard
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
        return motorRunning ? RPM : 0;
    }

    public double getMeasuredRPM() {
        return -(motor1.getVelocity() * 60.0) / 28;
    }

    public void increaseRPM() {
        RPM += 100;
    }

    public void decreaseRPM() {
        RPM -= 100;
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