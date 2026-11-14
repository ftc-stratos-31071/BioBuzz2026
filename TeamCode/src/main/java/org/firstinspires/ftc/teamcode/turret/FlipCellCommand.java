package org.firstinspires.ftc.teamcode.turret;

import com.seattlesolvers.solverslib.command.CommandBase;

// press after every hive tip so the turret switches to the other cell
public class FlipCellCommand extends CommandBase {
    private final HiveSubsystem hive;

    public FlipCellCommand(HiveSubsystem hive) {
        this.hive = hive;
        addRequirements(hive);
    }

    @Override
    public void initialize() {
        hive.flip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
