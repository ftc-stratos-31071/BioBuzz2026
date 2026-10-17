package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final ServoEx servo;
    public final MotorEx motor;
    public static double RPM = 500;
    public static boolean motorRunning = false;
    public double servoAngle = 0;
    public static double a_P = 0.002;
    public static double b_I = 0;
    public static double c_D = 0.01;
    public static double d_F = 0.0003;
    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;
    private Timing.Stopwatch timer;
    public ShooterSubsystem(final HardwareMap hMap, final String servoName, final String motorName, Timing.Stopwatch timer) {
        servo = new ServoEx(hMap, servoName, 180);

        motor = new MotorEx(hMap, motorName);
        motor.setRunMode(MotorEx.RunMode.RawPower);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        motor.resetEncoder();

//        motor.setInverted(true);

        this.timer = timer;
    }

    @Override
    public void periodic() {
        double error = RPM - getMeasuredRPM();

        double elapsed = timer.elapsedTime();
        double deltaTime = elapsed - lastTime;
        lastTime = elapsed;

        integral += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;
////
////        System.out.println(d_F * RPM);
////        System.out.println(a_P * error);
////        System.out.println(b_I * integral);
//        System.out.println(c_D * derivative);
        double output = d_F * RPM + a_P * error + b_I * integral + c_D * derivative;
        System.out.println(output);

//         called once per scheduler loop, use for checking things constantly
        if (motorRunning) {
            motor.set(output);
        } else {
            motor.stopMotor();
        }
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
        return -(motor.getVelocity() * 60.0) / 28;
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
        servo.set(servoAngle);
    }

    public void decreaseHoodAngle() {
        servoAngle -= 10;
        servo.set(servoAngle);
    }
}