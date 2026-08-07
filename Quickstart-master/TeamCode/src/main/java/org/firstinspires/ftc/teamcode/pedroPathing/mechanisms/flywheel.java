package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class flywheel{

    private DcMotorEx outtake;
    private DcMotorEx outtake2;




    double velocity;
    double kV = 0.00036;
    double P = 0.005;
    double kS = 0.08;
    double kV2 = 0.00038;
    double P2 = 0.006;


    int step = 1;
    public void init(HardwareMap hwMap) {
        outtake = hwMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hwMap.get(DcMotorEx.class, "o2");

        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);


    }


    public void shoot(double targetVelocity) {

        if (targetVelocity < 2000) {
            velocity = (outtake.getVelocity());
            double error = targetVelocity - velocity;
            double feedback = error * P;
            double feedforward = kV * targetVelocity + kS;
            outtake.setPower(feedback + feedforward);
            outtake2.setPower(feedback + feedforward);
        }
        else {
            velocity = (outtake.getVelocity());
            double error = targetVelocity - velocity;
            double feedback = error * P2;
            double feedforward = kV2 * targetVelocity + kS;
            outtake.setPower(feedback + feedforward);
            outtake2.setPower(feedback + feedforward);
        }





    }
    public double getVel(){
        return outtake.getVelocity();
    }
    public double autoshoot(double goalDistance) {
        return MathFunctions.clamp(
                0.0000261133 * Math.pow(goalDistance, 4)
                        - 0.00952655 * Math.pow(goalDistance, 3)
                        + 1.24773 * Math.pow(goalDistance, 2)
                        - 61.90251 * goalDistance
                        + 2490.82746,
                0,
                2520
        );    }




}
