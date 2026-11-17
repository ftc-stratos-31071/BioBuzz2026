package org.firstinspires.ftc.teamcode.intake;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;
    public static double test = 0;

    @Override
    public void periodic() {
        System.out.println(test);
    }

    public IntakeSubsystem(final HardwareMap hMap, final String intakeID) {
        intake = new Motor(hMap, intakeID);
        intake.setInverted(true);
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