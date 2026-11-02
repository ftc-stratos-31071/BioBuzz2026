package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;

public class TransferOutOffCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final IntakeSubsystem intake;

    public TransferOutOffCommand(TransferSubsystem t, IntakeSubsystem i) {
        transfer = t;
        intake = i;
        addRequirements(t);
        addRequirements(i);
    }

    @Override
    public void initialize() {
        transfer.setTransferOut(false);
        intake.setOuttake(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
