package org.firstinspires.ftc.teamcode.turret;

import static org.firstinspires.ftc.teamcode.turret.TurretConstants.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

// keeps track of which of our cells is facing up
public class HiveSubsystem extends SubsystemBase {
    private boolean farCellUp = !RED_ALLIANCE; // red starts with the audience-side cell up, blue with the far one

    public void flip() {
        farCellUp = !farCellUp;
    }

    public double targetX() {
        return RED_ALLIANCE ? -CELL_X : CELL_X;
    }

    public double targetY() {
        return farCellUp ? CELL_Y : -CELL_Y;
    }
}
