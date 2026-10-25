package org.firstinspires.ftc.teamcode.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

// command doesn't finish until button is released
public class IntakeOffCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeOffCommand(IntakeSubsystem i) {
        intake = i;
        addRequirements(i);
    }

    @Override
    public void initialize() {
        intake.setIntake(false);
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}