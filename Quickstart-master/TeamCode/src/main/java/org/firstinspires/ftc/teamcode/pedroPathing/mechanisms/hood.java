package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class hood {
    private Servo hood;
    public static double open = 0.34;
    public static double close = 0.2;
    public static double hoodpos ;

    public void init(HardwareMap hwMap) {
        hood = hwMap.get(Servo.class, "hood");


    }

    public double autoshoot(double x) {
        return MathFunctions.clamp(
                7.06855 * Math.pow(0.94747, x),
                0.1,
                0.8
        );

    }
    public void setPosition(double position){hood.setPosition(position);
    }


}
