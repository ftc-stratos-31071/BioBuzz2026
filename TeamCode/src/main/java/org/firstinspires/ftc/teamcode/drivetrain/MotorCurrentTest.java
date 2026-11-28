package org.firstinspires.ftc.teamcode.drivetrain;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Motor Current Test", group="Test")
public class MotorCurrentTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        String[] names = {"FL", "FR", "BL", "BR"};
        DcMotorEx[] motors = new DcMotorEx[4];

        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, names[i]);
        }
        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.REVERSE);
        motors[3].setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            boolean[] on = {
                    gamepad1.dpad_up || gamepad1.x,
                    gamepad1.dpad_up || gamepad1.y,
                    gamepad1.dpad_up || gamepad1.a,
                    gamepad1.dpad_up || gamepad1.b
            };

            for (int i = 0; i < 4; i++) {
                motors[i].setPower(on[i] ? 1 : 0);

                double amps = motors[i].getCurrent(CurrentUnit.AMPS);
                double speed = motors[i].getVelocity();

                telemetry.addData(names[i], "%.2f A   %.0f ticks/s", amps, speed);
                panels.addData(names[i] + " amps", amps);
                panels.addData(names[i] + " speed", speed);
            }

            telemetry.update();
            panels.update();
        }
    }
}
