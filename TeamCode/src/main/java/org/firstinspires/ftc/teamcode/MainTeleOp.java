package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.Timing;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.door.*;
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
    private DoorSubsystem door;
    private IntakeSubsystem intake;
    private GamepadEx gamepad;
    public Timing.Stopwatch timer;
    private LocalizationSubsystem localization;
    private AutoAimSubsystem autoAim;

    @Override
    public void initialize() {
        timer = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
        timer.start();

        gamepad = new GamepadEx(gamepad1);

        // SHOOTER
        shooter = new ShooterSubsystem(hardwareMap, "shooterL", timer);

        // turn shooter on at the start of teleop (it then stays on)
        GamepadButton shootButton = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        shootButton.whenPressed(new ShootCommand(shooter));

        // rpm trim: nudges the auto-aim rpm up/down by 100 (acts as plain rpm +/- while auto-aim is off)
        GamepadButton increaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_RIGHT);
        increaseRPMButton.whenPressed(new IncreaseRPMCommand(shooter));

        GamepadButton decreaseRPMButton = new GamepadButton(gamepad, GamepadKeys.Button.DPAD_LEFT);
        decreaseRPMButton.whenPressed(new DecreaseRPMCommand(shooter));

        // TURRET
        turret = new TurretSubsystem(hardwareMap, "turretRight", "turretLeft", "turretEncoder");

        // LOCALIZATION
        localization = new LocalizationSubsystem(hardwareMap, "pinpoint", 0, 0, 0, false);
        GamepadButton resetLocationButton = new GamepadButton(gamepad, GamepadKeys.Button.OPTIONS);
        resetLocationButton.whenPressed(new ResetPositionCommand(localization));

//        // AUTO AIM - must be created after localization, shooter and turret
        autoAim = new AutoAimSubsystem(localization, shooter, turret);
        // manual fallback: turns auto-aim off/on (turret and shooter then keep their last commands)
        GamepadButton autoAimToggleButton = new GamepadButton(gamepad, GamepadKeys.Button.BACK);
        autoAimToggleButton.whenPressed(new InstantCommand(autoAim::toggleEnabled));

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

        // DOOR - closes when intaking (L1), opens when feeding the shooter (cross)
//        door = new DoorSubsystem(hardwareMap, "door");
//        intakeButton.whenPressed(new DoorCloseCommand(door));

        // TRANSFER - all transfer actions do equivalent for intake - they run in parallel
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap, "transfer");

        // transfer+intake on/off
        GamepadButton transferButton = new GamepadButton(gamepad, GamepadKeys.Button.A);
        transferButton.whenPressed(new TransferOnCommand(transfer, intake));
        transferButton.whenReleased(new TransferOffCommand(transfer, intake));
//        transferButton.whenPressed(new DoorOpenCommand(door));

        // transfer+intake out on/off
        GamepadButton transferOutButton = new GamepadButton(gamepad, GamepadKeys.Button.B);
        transferOutButton.whenPressed(new TransferOutOnCommand(transfer, intake));
        transferOutButton.whenReleased(new TransferOutOffCommand(transfer, intake));

    }
}