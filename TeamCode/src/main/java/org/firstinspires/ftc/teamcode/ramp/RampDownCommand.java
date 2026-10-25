package org.firstinspires.ftc.teamcode.ramp;
import com.seattlesolvers.solverslib.command.CommandBase;
public class RampDownCommand extends CommandBase {
    private final RampSubsystem ramp;
    public RampDownCommand(RampSubsystem r) {
        ramp = r;
        addRequirements(r);
    }

    @Override
    public void initialize() {
        ramp.setRamp(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}