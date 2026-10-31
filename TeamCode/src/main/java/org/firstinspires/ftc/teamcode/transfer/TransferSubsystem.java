package org.firstinspires.ftc.teamcode.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class TransferSubsystem extends SubsystemBase {
    private final Motor transfer;

    public TransferSubsystem(final HardwareMap hMap, final String transferID) {
        transfer = new Motor(hMap, transferID);
    }

    public void setTransfer(boolean power) {
        if (power) transfer.set(1);
        else transfer.set(0);
    }

    public void setTransferOut(boolean power) {
        if (power) transfer.set(-1);
        else transfer.set(0);
    }
}
