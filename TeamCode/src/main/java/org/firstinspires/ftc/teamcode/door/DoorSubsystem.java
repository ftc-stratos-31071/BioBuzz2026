package org.firstinspires.ftc.teamcode.door;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class DoorSubsystem extends SubsystemBase {
    private final ServoImplEx door;

    public static double OPEN_POSITION = 0.0;
    public static double CLOSED_POSITION = 0.19;

    public DoorSubsystem(final HardwareMap hMap, final String doorID) {
        door = hMap.get(ServoImplEx.class, doorID);
        door.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void open() {
        door.setPosition(OPEN_POSITION);
    }

    public void close() {
        door.setPosition(CLOSED_POSITION);
    }
}
