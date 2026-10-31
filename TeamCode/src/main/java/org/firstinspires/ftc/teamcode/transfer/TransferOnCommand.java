package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

public class TransferOnCommand extends CommandBase {
    private final TransferSubsystem transfer;

    public TransferOnCommand(TransferSubsystem t) {
        transfer = t;
        addRequirements(t);
    }

    @Override
    public void initialize() {
        transfer.setTransfer(true);
    }

    @Override
    public boolean isFinished() {
        return true; // ends immediately, motor keeps its power until changed
    }
}
