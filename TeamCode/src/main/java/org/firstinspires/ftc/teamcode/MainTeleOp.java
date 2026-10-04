package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {

    private ShooterSubsystem shooter;
    private GamepadEx gamepad;

    @Override
    public void initialize() {
        gamepad = new GamepadEx(gamepad1);
        shooter = new ShooterSubsystem(hardwareMap, "hood", "shooter");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // any telemetry updates will be sent to both telemetry and ftc dashboard - like hdmi splitter

        GamepadButton shootButton = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        shootButton.whenPressed(new ShootCommand(shooter));

        GamepadButton increaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        increaseRPMButton.whenPressed(new IncreaseRPMCommand(shooter));

        GamepadButton decreaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        decreaseRPMButton.whenPressed(new DecreaseRPMCommand(shooter));

        // runs every loop of scheduler
        schedule(new RunCommand(() -> {
            telemetry.addData("Running", shooter.isRunning());
            telemetry.addData("Target RPM", shooter.getTargetRPM());
            telemetry.addData("Measured RPM", shooter.getMeasuredRPM());
            telemetry.update();
        }));
    }
}