package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;

public class IncreaseRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public IncreaseRPMCommand(ShooterSubsystem s) {
        shooter = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        shooter.increaseRPM();
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}