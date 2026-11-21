package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.localization.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.turret.TurretSubsystem;

/**
 * Glue between odometry and the mechanisms. Owns no hardware; every loop it:
 *   1. reads the robot pose,
 *   2. decides which cell is the target (HiveTargetSelector, from which half of the field the robot is in),
 *   3. solves the shot (ShotCalculator),
 *   4. points the turret at the target,
 *   5. updates the shooter's target RPM (the flywheel itself only spins once ShootCommand has toggled it on).
 *
 * Construct it AFTER the localization, shooter and turret subsystems so its periodic() runs after the pose update.
 */
@Configurable
public class AutoAimSubsystem extends SubsystemBase {

    // Panels switches so you can bring things up one at a time.
    // Both default to OFF so nothing changes until you've calibrated the turret and filled in the rpm table:
    //   CONTROL_TURRET: turn on after the turret calibration steps
    //   CONTROL_SHOOTER: turn on after ShotCalculator's TABLE has real rpm values (otherwise it commands 0 rpm)
    public static boolean CONTROL_TURRET = false;
    public static boolean CONTROL_SHOOTER = false;

    private final LocalizationSubsystem localization;
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final HiveTargetSelector selector = new HiveTargetSelector();

    private boolean enabled = true;

    public AutoAimSubsystem(LocalizationSubsystem localization, ShooterSubsystem shooter, TurretSubsystem turret) {
        this.localization = localization;
        this.shooter = shooter;
        this.turret = turret;
    }

    @Override
    public void periodic() {
        // manual fallback: if auto-aim is off (or odometry is unhealthy) leave the turret/shooter alone
        if (!enabled || !localization.isReady()) return;

        double x = localization.getX();
        double y = localization.getY();
        double heading = localization.getHeadingRad();

        selector.update(y);
        ShotCalculator.Solution sol = ShotCalculator.solve(x, y, heading, selector.getSide());

        if (CONTROL_TURRET) {
            turret.setAimAngleDeg(Math.toDegrees(sol.relativeAngleRad));
        }
        if (CONTROL_SHOOTER) {
            shooter.setTargetRPM(sol.rpm);
        }

        PanelsTelemetry.INSTANCE.getTelemetry().addData("aim distance (in)", sol.distanceIn);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("aim relative angle (deg)", Math.toDegrees(sol.relativeAngleRad));
        PanelsTelemetry.INSTANCE.getTelemetry().addData("aim rpm", sol.rpm);
        PanelsTelemetry.INSTANCE.getTelemetry().addData("aim target is HIGH_Y (1/0)", selector.getSide() == HiveTargetSelector.Side.HIGH_Y ? 1 : 0);
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }

    public void setEnabled(boolean on) {
        enabled = on;
    }

    public void toggleEnabled() {
        enabled = !enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }
}
