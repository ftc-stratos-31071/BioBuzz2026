package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;
    public IntakeSubsystem(final HardwareMap hMap, final String intakeID) {
        intake = new Motor(hMap, intakeID);
    }

    public void setIntake(boolean power) { // toggle intake according to boolean "power"
        if (power) intake.set(1);
        else intake.set(0);
    }
    public void setOuttake(boolean power) {
        if (power) intake.set(-1);
        else intake.set(0);
    }
}