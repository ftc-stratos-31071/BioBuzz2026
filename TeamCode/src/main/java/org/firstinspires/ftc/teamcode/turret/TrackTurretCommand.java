package org.firstinspires.ftc.teamcode.turret;

import static org.firstinspires.ftc.teamcode.turret.TurretConstants.*;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.localization.LocalizationSubsystem;

public class TrackTurretCommand extends CommandBase {
    private final LocalizationSubsystem localization;

    public TrackTurretCommand(TurretSubsystem turret, LocalizationSubsystem localization) {
        this.localization = localization;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // pinpoint
        double x = localization.getX();
        double y = localization.getY();
        double[] target = targetFor(x, y);

        double aim = Math.toDegrees(Math.atan2(target[1] - y, target[0] - x) - localization.getHeadingRad());
        aim = Math.max(MIN_DEG, Math.min(MAX_DEG, wrap(aim)));

        double command = (REVERSED ? -aim : aim) + FORWARD_DEG;
        TurretSubsystem.angle = ((command % 360) + 360) % 360; // keeps ts between 0 and 360
    }

    private double[] targetFor(double x, double y) {
        // Q1 = top right, Q2 = top left, Q3 = bottom left, Q4 = bottom right
        if (x >= 0) return y >= 0 ? new double[]{Q1_X, Q1_Y} : new double[]{Q4_X, Q4_Y};
        return y >= 0 ? new double[]{Q2_X, Q2_Y} : new double[]{Q3_X, Q3_Y};
    }

    private double wrap(double deg) {
        // keeps angle between -180 and 180
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    @Override
    public boolean isFinished() {
        return false; // never ends
    }
}
