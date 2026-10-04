package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;

public class IncreaseHoodAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public IncreaseHoodAngleCommand(ShooterSubsystem s) {
        shooter = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        shooter.toggleMotor();
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}