package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.drivetrain.*;
import org.firstinspires.ftc.teamcode.intake.*;
import org.firstinspires.ftc.teamcode.localization.*;
import org.firstinspires.ftc.teamcode.ramp.*;
import org.firstinspires.ftc.teamcode.transfer.*;
import org.firstinspires.ftc.teamcode.shooter.*;
import org.firstinspires.ftc.teamcode.turret.*;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {

    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private DrivetrainSubsystem drivetrain;
    private RampSubsystem ramp;
    private IntakeSubsystem intake;
    private GamepadEx gamepad;
    public Timing.Stopwatch timer;
    private LocalizationSubsystem localization;

    @Override
    public void initialize() {
        timer = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
        timer.start();

        gamepad = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // any telemetry updates will be sent to both telemetry and ftc dashboard - like hdmi splitter

        // SHOOTER
//        shooter = new ShooterSubsystem(hardwareMap, "hood", "shooter", timer);
//
//        // toggle shooter
//        GamepadButton shootButton = new GamepadButton(gamepad, GamepadKeys.Button.Y);
//        shootButton.whenPressed(new ShootCommand(shooter));
//
//        // increase rpm
//        GamepadButton increaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
//        increaseRPMButton.whenPressed(new IncreaseRPMCommand(shooter));
//
//        // decrease rpm
//        GamepadButton decreaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
//        decreaseRPMButton.whenPressed(new DecreaseRPMCommand(shooter));
//
//        // increase hood angle
//        GamepadButton increaseHoodAngleButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP);
//        increaseHoodAngleButton.whenPressed(new IncreaseHoodAngleCommand(shooter));
//
//        // decrease hood angle
//        GamepadButton decreaseHoodAngleCommand = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN);
//        decreaseHoodAngleCommand.whenPressed(new DecreaseHoodAngleCommand(shooter));

        // TURRET
//        turret = new TurretSubsystem(hardwareMap, "turretRight", "turretLeft", "turretEncoder");

        // LOCALIZATION
        localization = new LocalizationSubsystem(hardwareMap, "pinpoint", 0, 0, 0, false);
        GamepadButton resetLocationButton = new GamepadButton(gamepad, GamepadKeys.Button.OPTIONS);
        resetLocationButton.whenPressed(new ResetPositionCommand(localization));

        // DRIVETRAIN
        drivetrain = new DrivetrainSubsystem(hardwareMap, "FL", "FR", "BL", "BR", gamepad, localization);
        // all controls already defined and checked for within class

        // RAMP
        ramp = new RampSubsystem(hardwareMap, "ramp");
        GamepadButton rampButton = new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER);
        rampButton.whenPressed(new RampDownCommand(ramp));
        rampButton.whenReleased(new RampUpCommand(ramp));

        // INTAKE - run intake independent of transfer
        intake = new IntakeSubsystem(hardwareMap, "intake");

        // intake on/off
        GamepadButton intakeButton = new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER);
        intakeButton.whenPressed(new IntakeOnCommand(intake));
        intakeButton.whenReleased(new IntakeOffCommand(intake));

        // TRANSFER - all transfer actions do equivalent for intake - they run in parallel
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, "transfer");

        // transfer+intake on/off
        GamepadButton transferButton = new GamepadButton(gamepad, GamepadKeys.Button.A);
        transferButton.whenPressed(new TransferOnCommand(transfer, intake));
        transferButton.whenReleased(new TransferOffCommand(transfer, intake));

        // transfer+intake out on/off
        GamepadButton transferOutButton = new GamepadButton(gamepad, GamepadKeys.Button.B);
        transferOutButton.whenPressed(new TransferOutOnCommand(transfer, intake));
        transferOutButton.whenReleased(new TransferOutOffCommand(transfer, intake));
    }
}