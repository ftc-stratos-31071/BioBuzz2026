package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;

public class TransferOffCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final IntakeSubsystem intake;

    public TransferOffCommand(TransferSubsystem t, IntakeSubsystem i) {
        transfer = t;
        intake = i;
        addRequirements(t);
        addRequirements(i);
    }

    @Override
    public void initialize() {
        transfer.setTransfer(false);
        intake.setIntake(false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
