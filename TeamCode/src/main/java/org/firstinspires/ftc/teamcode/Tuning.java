package org.firstinspires.ftc.teamcode;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

// tuners show up at 192.168.43.1:10158 when connected to the robot wifi
public class Tuning {
    @Tuner(name = "1. Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::localizer, Constants::drivetrain);
    }

    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(Constants::drivetrain, Constants::localizer, Constants::algorithm);
    }
}
