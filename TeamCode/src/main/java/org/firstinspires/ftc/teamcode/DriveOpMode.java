package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class DriveOpMode extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor intake;
    private DcMotor shootright;
    private DcMotor shootleft;
    private DcMotor transfer;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        shootleft = hardwareMap.get(DcMotor.class, "shootleft");
        shootright = hardwareMap.get(DcMotor.class, "shootright");

        boolean shooting = false;
        boolean lastrightbumper = false;

        // reversing right motors
//        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backleft.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // DRIVE:
            // left stick - driving/strafing, right stick - turning
            double y = -gamepad1.left_stick_y;  // y stick is reversed
            double x = gamepad1.left_stick_x;   // strafing
            double rx = gamepad1.right_stick_x; // rotating

            // calculate motor speeds
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontleftPower = (y + x + rx) / denominator;
            double backleftPower = (y - x + rx) / denominator;
            double frontrightPower = (y - x - rx) / denominator;
            double backrightPower = (y + x - rx) / denominator;

            frontleft.setPower(frontleftPower);
            backleft.setPower(backleftPower);
            frontright.setPower(frontrightPower);
            backright.setPower(backrightPower);

            // INTAKE:
            // forward intake
            if (gamepad1.left_bumper) {
                intake.setPower(1);
                // reverse intake
            } else if (gamepad1.b) {
                intake.setPower(-1);
                transfer.setPower(-1); // don't need the setPower(0) line in the else statement here because in the next if block it already sets it to 0
            } else {
                intake.setPower(0);
            }

            // TRANSFER + SHOOTING:
            // check if shooter should be stopped or started
            if (gamepad1.a) {
                transfer.setPower(1);
            } else {
                transfer.setPower(0);
            }

            if (gamepad1.right_bumper != lastrightbumper) {
                // toggle off or on
                shooting = !shooting;

                lastrightbumper = gamepad1.right_bumper;

                // change motor powers according to new value of "shooting"
                if (shooting) {
                    shootright.setPower(0.5);
                    shootleft.setPower(-0.5);
                } else {
                    shootright.setPower(0);
                    shootleft.setPower(0);
                }
            }
        }
    }
}
}