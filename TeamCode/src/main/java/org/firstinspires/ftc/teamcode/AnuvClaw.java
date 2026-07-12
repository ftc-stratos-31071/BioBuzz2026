package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Control")
public class AnuvClaw extends OpMode {

    Servo clawServo;

    static final double OPEN   = 0.0;
    static final double CLOSED = 0.45;

    @Override
    public void init() {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo.setPosition(CLOSED);
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            clawServo.setPosition(OPEN);
        }
        if (gamepad1.b) {
            clawServo.setPosition(CLOSED);
        }

    }
}