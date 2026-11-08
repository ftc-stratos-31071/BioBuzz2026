package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public static double RPM = 4000;
    public static boolean motorRunning = false;
    public static double servoAngle = 0;
    public double measuredRPM;

    // PIDF variables for flywheel:
    public static double a_P = 0.0005;
    public static double b_I = 0;
    public static double c_D = 0;
    public static double d_F = 0.00023;

    // tuning helpers (change live in FTC Dashboard):
    public static boolean MOTOR2_REVERSED = false; // flip if the two flywheel motors fight each other
    public static boolean RPM_REVERSED = true;     // flip if "shooter rpm" reads negative while spinning forward
    private double output = 0;                     // power actually sent to the motors (for telemetry)
    private boolean wasRunning = false;
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
            motor2.setInverted(MOTOR2_REVERSED);

            // flywheel just turned on: clear old PID memory so it doesn't jump
            if (!wasRunning) {
                integral = 0;
                lastError = RPM - getMeasuredRPM(); // avoids a derivative spike on the first loop
                lastTime = timer.elapsedTime();
                wasRunning = true;
            }

            // PIDF STUFF:
            double error = RPM - getMeasuredRPM();

            double elapsed = timer.elapsedTime();
            double deltaTime = elapsed - lastTime;
            if (deltaTime == 0) deltaTime = 0.0001; // just in case so that we don't get undefined results if two loops run on same millisecond and therefore delta time = 0
            lastTime = elapsed;

            integral += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            output = d_F * RPM + a_P * error + b_I * integral + c_D * derivative;
            output = Math.max(-1, Math.min(1, output)); // motors only accept -1 to 1

            motor1.set(-output);
            motor2.set(output);
        } else {
            motor1.stopMotor();
            motor2.stopMotor();
            timer.pause();
            output = 0;
            wasRunning = false;
        }

        measuredRPM = getMeasuredRPM(); // so i can see current rpm in ftc dashboard

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("shooter rpm: ", measuredRPM);
        packet.put("intended rpm: ", RPM);
        packet.put("shooter power: ", output); // stuck at 1.0 = maxed out, something is wrong
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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
        double rpm = (motor1.getVelocity() * 60.0) / 28;
        return RPM_REVERSED ? -rpm : rpm;
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