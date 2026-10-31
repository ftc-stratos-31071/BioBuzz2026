package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

public class TransferOutOnCommand extends CommandBase {
    private final TransferSubsystem transfer;

    public TransferOutOnCommand(TransferSubsystem t) {
        transfer = t;
        addRequirements(t);
    }

    @Override
    public void initialize() {
        transfer.setTransferOut(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
