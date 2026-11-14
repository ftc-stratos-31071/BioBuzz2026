package org.firstinspires.ftc.teamcode.turret;

import static org.firstinspires.ftc.teamcode.turret.TurretConstants.*;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.localization.LocalizationSubsystem;

// keeps the turret pointed at our up cell the whole match
public class TrackTurretCommand extends CommandBase {
    private final LocalizationSubsystem localization;
    private final HiveSubsystem hive;

    public TrackTurretCommand(TurretSubsystem turret, LocalizationSubsystem localization, HiveSubsystem hive) {
        this.localization = localization;
        this.hive = hive;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double dx = hive.targetX() - localization.getX();
        double dy = hive.targetY() - localization.getY();

        // minus heading so it stays locked on while the robot turns
        double aim = Math.toDegrees(Math.atan2(dy, dx) - localization.getHeadingRad());
        aim = Math.max(MIN_DEG, Math.min(MAX_DEG, wrap(aim)));

        double command = (REVERSED ? -aim : aim) + FORWARD_DEG;
        TurretSubsystem.angle = ((command % 360) + 360) % 360; // keeps it between 0 and 360
    }

    // keeps angle between -180 and 180
    private double wrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    @Override
    public boolean isFinished() {
        return false; // never ends
    }
}
