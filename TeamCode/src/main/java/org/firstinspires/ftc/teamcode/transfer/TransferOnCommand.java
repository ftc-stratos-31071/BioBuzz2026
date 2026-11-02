package org.firstinspires.ftc.teamcode.transfer;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.intake.IntakeSubsystem;

public class TransferOnCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final IntakeSubsystem intake;

    public TransferOnCommand(TransferSubsystem t, IntakeSubsystem i) {
        transfer = t;
        intake = i;
        addRequirements(t);
        addRequirements(i);
    }

    @Override
    public void initialize() {
        transfer.setTransfer(true);
        intake.setIntake(true);
    }

    @Override
    public boolean isFinished() {
        return true; // ends immediately, motor keeps its power until changed
    }
}
