package org.firstinspires.ftc.teamcode.door;

import com.seattlesolvers.solverslib.command.CommandBase;

public class DoorOpenCommand extends CommandBase {
    private final DoorSubsystem door;

    public DoorOpenCommand(DoorSubsystem d) {
        door = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {
        door.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
