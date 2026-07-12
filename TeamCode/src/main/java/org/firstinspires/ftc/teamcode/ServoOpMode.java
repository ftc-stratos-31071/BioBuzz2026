package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.ServoConfig;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class ServoOpMode extends OpMode {
    ServoConfig servo = new ServoConfig();
    @Override
    public void init() {
        servo.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.b) {
        // right button
            if (servo.position < 1) {
                servo.position += 0.005;
            }
        }
        else if (gamepad1.x) {
        // left button
            if (servo.position > 0) {
                servo.position -= 0.005;
            }
        }

        servo.setServoPosition(servo.position);
    }
}
