package org.firstinspires.ftc.teamcode.localization;

import com.seattlesolvers.solverslib.command.CommandBase;

public class ResetPositionCommand extends CommandBase {
    private final LocalizationSubsystem localizationSubsystem;
    public ResetPositionCommand(LocalizationSubsystem l) {
        localizationSubsystem = l;
        addRequirements(l);
    }

    @Override
    public void initialize() {
        localizationSubsystem.setPose(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}