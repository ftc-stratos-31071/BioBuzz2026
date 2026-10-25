package org.firstinspires.ftc.teamcode.ramp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
@Config
public class RampSubsystem extends SubsystemBase {
    private final Servo ramp;

    // gotta tune ts (complete guess)
    public static double UP_POSITION = 0.6;
    public static double DOWN_POSITION = 0.4;
    private boolean isUp = true;

    public RampSubsystem(final HardwareMap hMap, final String rampID) {
        ramp = hMap.get(Servo.class, rampID);
        setRamp(true);
    }
    public void setRamp(boolean up) {
        isUp = up;
        if (up) {
            ramp.setPosition(UP_POSITION);
        } else {
            ramp.setPosition(DOWN_POSITION);
        }
    }
    public void toggle() {
        setRamp(!isUp);
    }
    public boolean isUp() {
        return isUp;
    }
}