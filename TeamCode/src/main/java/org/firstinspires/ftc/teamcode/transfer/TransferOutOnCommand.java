package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;

public class TransferOutOnCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final IntakeSubsystem intake;

    public TransferOutOnCommand(TransferSubsystem t, IntakeSubsystem i) {
        transfer = t;
        intake = i;
        addRequirements(t);
        addRequirements(i);
    }

    @Override
    public void initialize() {
        transfer.setTransferOut(true);
        intake.setOuttake(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
