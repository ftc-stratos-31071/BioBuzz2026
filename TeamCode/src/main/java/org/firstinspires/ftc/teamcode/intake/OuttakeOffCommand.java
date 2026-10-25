package org.firstinspires.ftc.teamcode.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

// command doesn't finish until button is released
public class OuttakeOffCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public OuttakeOffCommand(IntakeSubsystem i) {
        intake = i;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        intake.setOuttake(false);
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}