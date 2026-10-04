package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final ServoEx servo;
    public final MotorEx motor;
    public static double RPM = 500;
    public static boolean motorRunning = false;
    public double servoAngle = 0;
    public ShooterSubsystem(final HardwareMap hMap, final String servoName, final String motorName) {
        servo = new ServoEx(hMap, servoName, 180);

        motor = new MotorEx(hMap, motorName);
        motor.setRunMode(MotorEx.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);

        motor.resetEncoder();
    }

    @Override
    public void periodic() {
        // called once per scheduler loop, use for checking things constantly
        if (motorRunning) {
            motor.setVelocity((RPM * 28) / 60.0); // ASK ANUV IF WE HAVE ANY GEARBOX OUTSIDE THE MOTOR
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
        return (motor.getVelocity() * 60.0) / 28;
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