package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;

public class DecreaseHoodAngleCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public DecreaseHoodAngleCommand(ShooterSubsystem s) {
        shooter = s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        shooter.decreaseHoodAngle();
    }

    @Override
    public boolean isFinished() {
        return true; // Ends immediately after toggling
    }
}