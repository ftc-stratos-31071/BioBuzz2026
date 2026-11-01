package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

public class TransferOutOffCommand extends CommandBase {
    private final TransferSubsystem transfer;

    public TransferOutOffCommand(TransferSubsystem t) {
        transfer = t;
        addRequirements(t);
    }

    @Override
    public void initialize() {
        transfer.setTransferOut(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
