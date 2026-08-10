package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "Mecanum Drive")
public class MecanumDrive extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor extension;
    @Override
    public void runOpMode() {
        // motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        extension = hardwareMap.get(DcMotor.class, "extension");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // driving the robot
            double y  = -gamepad1.left_stick_y; // forward/backward
            double x  = gamepad1.left_stick_x; // strafe
            double rx = gamepad1.right_stick_x; // rotate

            // motor speed never > 1.0
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // wheel power
            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower((y - x + rx) / denominator);
            frontRight.setPower((y - x - rx) / denominator);
            backRight.setPower((y + x - rx) / denominator);

            // extension
            if (gamepad2.left_stick_y > 0.1) {
                extension.setPower(gamepad2.left_stick_y); // extend
            } else if (gamepad2.left_stick_y < -0.1) {
                extension.setPower(gamepad2.left_stick_y); // retract
            } else {
                extension.setPower(0);
            }
        }
    }
}