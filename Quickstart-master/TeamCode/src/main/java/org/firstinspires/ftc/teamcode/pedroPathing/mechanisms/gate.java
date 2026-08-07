package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class gate {
    private Servo gate;
    public static double open = 0.34;
    public static double close = 0.2;
    private double currentPosition = 0.5;

    public void init(HardwareMap hwMap) {
        gate = hwMap.get(Servo.class, "gate");


    }

    public void open() {
        gate.setPosition(open);
    }
    public void close() {
        gate.setPosition(close);
    }

}
