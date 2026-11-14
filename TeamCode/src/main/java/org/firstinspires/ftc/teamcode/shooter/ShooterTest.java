package org.firstinspires.ftc.teamcode.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {
    public static String MOTOR_NAME = "git add .\n" +
            "git commit -m \"added shooter test opmode, turret aims at up cell\"\n" +
            "git pull\n" +
            "git push";
    public static boolean REVERSED = false;
    public static double TICKS_PER_REV = 28;

    public static double RPM = 3000;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.0002;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        boolean running = false, lastTriangle = false, lastUp = false, lastDown = false;
        double integral = 0, lastError = 0;
        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.triangle && !lastTriangle) {
                running = !running;
                integral = 0;
                lastError = 0;
                timer.reset();
            }
            if (gamepad1.dpad_up && !lastUp) RPM += 100;
            if (gamepad1.dpad_down && !lastDown) RPM = Math.max(0, RPM - 100);
            lastTriangle = gamepad1.triangle;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            motor.setDirection(REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            double measured = motor.getVelocity() * 60.0 / TICKS_PER_REV;

            double power = 0;
            if (running) {
                double dt = Math.max(timer.seconds(), 0.001);
                timer.reset();
                double error = RPM - measured;
                integral += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;
                power = Math.max(-1, Math.min(1, kF * RPM + kP * error + kI * integral + kD * derivative));
            }
            motor.setPower(power);

            telemetry.addData("running", running);
            telemetry.addData("target rpm", RPM);
            telemetry.addData("measured rpm", measured);
            telemetry.addData("power", power);
            telemetry.update();
        }
        motor.setPower(0);
    }
}
