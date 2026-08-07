
package org.firstinspires.ftc.teamcode.pedroPathing.mechanisms;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;


@Configurable
@TeleOp
public class ServoTurretTest extends OpMode {
    private Servo turretA, turretB;
    public static double position = 0.5;
    private hood hood = new hood();
    private intake intake = new intake();
    private gate gate = new gate();

    double velocity;
    @Override
    public void init() {
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        hood.init(hardwareMap);
        //TODO: Set motor name and direction
        turretA = hardwareMap.get(Servo.class, "turretA");
        turretB = hardwareMap.get(Servo.class, "turretB");
        turretA.setDirection(Servo.Direction.FORWARD);
        turretB.setDirection(Servo.Direction.FORWARD);

        if (turretA.getController() instanceof ServoControllerEx) {
// Confirm its an extended range servo controller before we try to set to avoid crash
            telemetry.addLine("ex");
            telemetry.update();
            ServoControllerEx theControl = (ServoControllerEx) turretA.getController();
            int thePort = turretA.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(500, 2500);
            theControl.setServoPwmRange(thePort, theRange);
        }

        if (turretB.getController() instanceof ServoControllerEx) {
// Confirm its an extended range servo controller before we try to set to avoid crash
            telemetry.addLine("ex");
            telemetry.update();
            ServoControllerEx theControl = (ServoControllerEx) turretB.getController();
            int thePort = turretB.getPortNumber();
            PwmControl.PwmRange theRange = new PwmControl.PwmRange(500, 2500);
            theControl.setServoPwmRange(thePort, theRange);
        }

    }

    @Override
    public void loop() {

        turretA.setPosition(position);
        turretB.setPosition(position);


        if (gamepad1.a) {
            intake.intakeonly();
        }
        else {
            intake.stop();
        }
        if (gamepad1.b) {
            gate.open();
        }
        else {
            gate.close();
        }


    }
}