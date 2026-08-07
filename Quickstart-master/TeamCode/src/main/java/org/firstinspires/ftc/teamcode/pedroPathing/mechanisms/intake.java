package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {
    private DcMotor intake, intake2, o1, o2;
    private CRServo VectorLeft, VectorRight;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");
        intake2 = hwMap.get(DcMotor.class, "intake2");

        o1 = hwMap.get(DcMotor.class, "o1");
        o2 = hwMap.get(DcMotor.class, "o2");
//        VectorLeft = hwMap.get(CRServo.class, "left");
//        VectorRight = hwMap.get(CRServo.class, "right");


    }
    public void intakeonly(){
        intake.setPower(1);
        intake2.setPower(-0.6);
    }
    public void allspin(){
        intake.setPower(1);
        intake2.setPower(-1);
//        VectorLeft.setPower(-1);
//        VectorRight.setPower(1);
    }
    public void reverse(){
        intake.setPower(-1);
        intake2.setPower(1);
    }

    public void stop(){
        intake.setPower(0);
        intake2.setPower(0);
//        VectorLeft.setPower(0);
//        VectorRight.setPower(0);
    }
}
