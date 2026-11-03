package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; // if this import fails, copy goBILDA's GoBildaPinpointDriver.java into TeamCode and import that instead
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// uses gobilda pinpoint module: 2 dead wheels (field coords: inches) and imu for heading (radians)
// +x is forward and +y is left - this is gobilda pinpoint standards

@Config
public class LocalizationSubsystem extends SubsystemBase {

    // instructions for calculating offsets (& documentation for the pinpoint module): https://www.gobilda.com/content/user_manuals/3110-0002-0001_user-guide.pdf
    private static final double X_POD_OFFSET_MM = 31.325;
    private static final double Y_POD_OFFSET_MM = 65.857;

    // pose carried from Autonomous into TeleOp (must be static in order to be carried over from autonomous to teleop)
    private static double savedX = 0, savedY = 0, savedHeading = 0;
    private static boolean hasSaved = false;

    private final GoBildaPinpointDriver pinpoint;
    public static double x, y;
    private static double heading; // not public so that the static one can be the degrees version, the radians version is used for everything except human viewing
    public static double headingDeg;

    public LocalizationSubsystem(final HardwareMap hMap, final String name,
                                 double startXIn, double startYIn, double startHeadingRad,
                                 boolean useSavedPose) {
        pinpoint = hMap.get(GoBildaPinpointDriver.class, name);

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM); // older driver versions: setOffsets(x_mm, y_mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // test: pushing the robot forward must make X increase, pushing it left must make Y increase. make inversions here:
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        pinpoint.resetPosAndIMU(); // zeroes pose + recalibrates IMU gyro offset

        // resetting takes this long - MUST be stationary
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        double sx = startXIn, sy = startYIn, sh = startHeadingRad;
        if (useSavedPose && hasSaved) {
            sx = savedX;
            sy = savedY;
            sh = savedHeading;
        }
        setPose(sx, sy, sh);
    }

    @Override
    public void periodic() {
        pinpoint.update(); // data only refreshes when this is called
        readPose();
    }

    private void readPose() {
        Pose2D p = pinpoint.getPosition();
        x = p.getX(DistanceUnit.INCH);
        y = p.getY(DistanceUnit.INCH);
        heading = wrapRad(p.getHeading(AngleUnit.RADIANS));
        headingDeg = Math.round(heading * 180 / Math.PI);
    }

    // ---------------- getters ----------------
    public double getX() { return x; }                 // inches
    public double getY() { return y; }                 // inches
    public double getHeadingRad() { return heading; }  // radians, CCW+, 0 = field +X
    public double getHeadingDeg() { return Math.toDegrees(heading); }

    /** false while the Pinpoint is calibrating or reporting a fault; don't trust the pose when false. */
    public boolean isReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    // ---------------- pose control ----------------
    /** Overwrite the tracked pose (e.g. after driving into a corner and pressing a re-localize button). */
    public void setPose(double xIn, double yIn, double headingRad) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.RADIANS, headingRad));
        pinpoint.update();
        readPose();
    }

    // call at the end of Autonomous so TeleOp can pick up where the robot is
    public void savePose() {
        savedX = x;
        savedY = y;
        savedHeading = heading;
        hasSaved = true;
    }

    public static void clearSavedPose() {
        hasSaved = false;
    }

    public static double wrapRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}