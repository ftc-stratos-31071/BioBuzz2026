package org.firstinspires.ftc.teamcode.ramp;
import com.seattlesolvers.solverslib.command.CommandBase;
public class RampUpCommand extends CommandBase {
    private final RampSubsystem ramp;
    public RampUpCommand(RampSubsystem r) {
        ramp = r;
        addRequirements(r);
    }

    @Override
    public void initialize() {
        ramp.setRamp(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}