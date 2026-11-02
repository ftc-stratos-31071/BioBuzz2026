package org.firstinspires.ftc.teamcode.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Config
public class DrivetrainSubsystem extends SubsystemBase {
    private Motor frontleft;
    private Motor frontright;
    private Motor backleft;
    private Motor backright;
    private final GamepadEx gamepad;
    public static boolean braking;
    public DrivetrainSubsystem(final HardwareMap hMap, final String frontleftID, final String frontrightID, final String backleftID, final String backrightID, final GamepadEx gamepad) {
        frontleft = new Motor(hMap, frontleftID);
        frontright = new Motor(hMap, frontrightID);
        backleft = new Motor(hMap, backleftID);
        backright = new Motor(hMap, backrightID);

        frontleft.setInverted(true);
        backleft.setInverted(true);
        backright.setInverted(true);

        this.gamepad = gamepad;
    }

    @Override
    public void periodic() {
        // left stick - driving/strafing, right stick - turning
        double y = gamepad.getLeftY();  // y stick is reversed
        double x = gamepad.getLeftX();;   // strafing
        double rx = gamepad.getRightX(); // rotating

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontleftPower = (y + x + rx) / denominator;
        double backleftPower = (y - x + rx) / denominator;
        double frontrightPower = -1 * (y - x - rx) / denominator;
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