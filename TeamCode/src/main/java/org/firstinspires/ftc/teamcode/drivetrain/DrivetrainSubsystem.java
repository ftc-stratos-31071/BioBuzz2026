package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.localization.LocalizationSubsystem;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    private Motor frontleft;
    private Motor frontright;
    private Motor backleft;
    private Motor backright;
    private final GamepadEx gamepad;
    public static boolean braking;
    private LocalizationSubsystem localization;

    // stick-forward in field-centric mode drives the robot toward this direction.
    private double driverForwardRad = 0;
    public DrivetrainSubsystem(final HardwareMap hMap, final String frontleftID, final String frontrightID, final String backleftID, final String backrightID, final GamepadEx gamepad, final LocalizationSubsystem l) {
        frontleft = new Motor(hMap, frontleftID);
        frontright = new Motor(hMap, frontrightID);
        backleft = new Motor(hMap, backleftID);
        backright = new Motor(hMap, backrightID);

        frontleft.setInverted(true);
        backright.setInverted(true);
        backleft.setInverted(true);

        this.gamepad = gamepad;

        localization = l;
    }

    @Override
    public void periodic() {
        // left stick - driving/strafing, right stick - turning
        double y = gamepad.getLeftY();  // y stick is reversed
        double x = gamepad.getLeftX();;   // strafing
        double rx = gamepad.getRightX(); // rotating

        // rotate the stick vector by -heading so "forward" means field-forward, not robot-forward.
        // (heading is CCW-positive; x = right, y = forward on the sticks)
        double h = localization.getHeadingRad() - driverForwardRad;
        double cos = Math.cos(h);
        double sin = Math.sin(h);
        double fieldX = x * cos + y * sin;
        double fieldY = -x * sin + y * cos;
        x = fieldX;
        y = fieldY;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontleftPower = (y + x + rx) / denominator;
        double backleftPower = (y - x + rx) / denominator;
        double frontrightPower = (y - x - rx) / denominator;
        double backrightPower = (y + x - rx) / denominator;

        frontleft.set(frontleftPower);
        frontright.set(frontrightPower);
        backleft.set(backleftPower);
        backright.set(backrightPower);
    }

    public void setZeroPowerBraking(boolean braking) {
        this.braking = braking;
        if (braking) {
            frontleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            frontright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            backright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        } else {
            frontleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            frontright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            backleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            backright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        }
    }
}