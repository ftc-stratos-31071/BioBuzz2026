package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;

public class DecreaseRPMCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public DecreaseRPMCommand(ShooterSubsystem s) {
        shooter = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        shooter.decreaseRPM();
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}