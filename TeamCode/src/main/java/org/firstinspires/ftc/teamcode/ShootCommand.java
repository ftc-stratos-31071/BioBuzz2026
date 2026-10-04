package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.ShooterSubsystem;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem s) {
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