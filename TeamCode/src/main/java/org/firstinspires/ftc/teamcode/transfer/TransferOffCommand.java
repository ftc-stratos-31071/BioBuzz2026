package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

public class TransferOffCommand extends CommandBase {
    private final TransferSubsystem transfer;

    public TransferOffCommand(TransferSubsystem t) {
        transfer = t;
        addRequirements(t);
    }

    @Override
    public void initialize() {
        transfer.setTransfer(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
