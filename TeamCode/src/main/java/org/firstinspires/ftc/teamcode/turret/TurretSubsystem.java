package org.firstinspires.ftc.teamcode.turret;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretSubsystem extends SubsystemBase {
    private final CRServoEx servoR;
    private final CRServoEx servoL;
    private final CRServoGroup servos;
    private final AbsoluteAnalogEncoder encoder;
    public static double angle;

    public TurretSubsystem(final HardwareMap hMap, final String servoNameR, final String servoNameL, final String encoderName) {
        encoder = new AbsoluteAnalogEncoder(hMap, encoderName, 3.2, AngleUnit.DEGREES);

        servoR = new CRServoEx(hMap, servoNameR, encoder, CRServoEx.RunMode.OptimizedPositionalControl); // FIX: need to set pidf coefficients: crServoEx.setPIDF(new PIDFCoefficients(0.001, 0.0, 0.1, 0.0001));
        servoR.setPIDF(new PIDFCoefficients(0.001, 0.0, 0.1, 0.0001));

        servoL = new CRServoEx(hMap, servoNameL, encoder, CRServoEx.RunMode.OptimizedPositionalControl); // FIX: need to set pidf coefficients: crServoEx.setPIDF(new PIDFCoefficients(0.001, 0.0, 0.1, 0.0001));
        servoL.setPIDF(new PIDFCoefficients(0.001, 0.0, 0.1, 0.0001));

        servos = new CRServoGroup(servoR, servoL);
    }

    @Override
    public void periodic() {
        servos.set(angle % 360); // so if angle is set above 360 it wont try to go past 360 degrees
    }
}