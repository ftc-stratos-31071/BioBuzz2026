package org.firstinspires.ftc.teamcode.door;

import com.seattlesolvers.solverslib.command.CommandBase;

public class DoorCloseCommand extends CommandBase {
    private final DoorSubsystem door;

    public DoorCloseCommand(DoorSubsystem d) {
        door = d;
        addRequirements(d);
    }

    @Override
    public void initialize() {
        door.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
