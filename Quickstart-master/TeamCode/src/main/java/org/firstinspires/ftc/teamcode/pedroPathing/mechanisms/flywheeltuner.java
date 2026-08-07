
package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class flywheeltuner {
    private DcMotorEx outtake, outtake2;

    public static double P = 0.005;
    public static double kV = 0.00036;
    public static double kS = 0.08;



    double velocity;

    public void init(HardwareMap hwMap) {
        outtake = hwMap.get(DcMotorEx.class, "o1");
        outtake2 = hwMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void shoot(double targetVelocity) {

        velocity = (outtake.getVelocity());

        double error = targetVelocity - velocity;


            double feedback = error * P;
            double feedforward = kV * targetVelocity + kS;
            outtake.setPower(feedback + feedforward);
            outtake2.setPower(feedback + feedforward);


    }
}