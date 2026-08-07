package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheeltuner.P;
import static org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheeltuner.kS;
import static org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheeltuner.kV;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret2;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheeltuner;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.gate;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Configurable
@TeleOp(name = "blue", group = "TeleOp")
public class blue extends OpMode {

    // ============================================================================
    // MECHANISMS
    // ============================================================================

    private Follower follower;
    public double shootvel;
    public static double hoodpos;
    public static double vel;
    private hood hood = new hood();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ServoTurret2 turret = new ServoTurret2();
    private flywheel shooter = new flywheel();
    private boolean shoot = false;
    private intake intake = new intake();
    private gate gate = new gate();
    private DcMotorEx outtake, outtake2;

    // ============================================================================
    // SETTINGS
    // ============================================================================

    private static final double SLOW_MODE_FACTOR = 0.4;

    // ============================================================================
    // OPMODE LIFECYCLE
    // ============================================================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(50, 68, Math.toRadians(180)));
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        hood.init(hardwareMap);
        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        hoodpos = hood.autoshoot(turret.getDistanceToGoal());
        hood.setPosition(hoodpos);
        vel = (int) MathFunctions.clamp(
                4.81475 * turret.getDistanceToGoal() + 666.22753,
                0,
                1720
        );
        hood.setPosition(hoodpos);

        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        turret.update(follower);

        // ====================================================================
        // DRIVING
        // ====================================================================
        double forward = gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        follower.setTeleOpDrive(forward, strafe, rotate, false);
        follower.update();

        // ====================================================================
        // INTAKE
        // ====================================================================
        if (gamepad1.rightBumperWasPressed()) {
            if (shoot) {
                intake.allspin();
            } else {
                intake.intakeonly();
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            intake.reverse();
        } else if (gamepad1.rightBumperWasReleased() || gamepad1.dpadDownWasReleased()) {
            intake.stop();
        }

        if (gamepad1.aWasPressed()) {
            follower.setPose(new Pose(9.5, 7.2, Math.toRadians(180)));
        }

        if (gamepad1.bWasPressed()) {
            follower.setPose(new Pose(137, 7.2, Math.toRadians(180)));
        }

        // ====================================================================
        // FLYWHEEL + GATE
        // gamepad2 right bumper: spin up flywheel only (no gate)
        // gamepad1 left bumper:  open gate + spin up flywheel
        // Flywheel stays on as long as either bumper is held
        // ====================================================================
        boolean flywheelActive = gamepad1.left_bumper || gamepad2.left_bumper || gamepad2.right_bumper;
        if (flywheelActive) {
            shoot = true;
            shootvel = vel;
        } else {
            shoot = false;
            shootvel = 0;
        }

        // Gate controlled by gamepad1 OR gamepad2 left bumper
        if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
            gate.open();
        } else if (gamepad1.leftBumperWasReleased() || gamepad2.leftBumperWasReleased()) {
            gate.close();
        }

        if (shootvel == 0) {
            outtake.setPower(0);
            outtake2.setPower(0);
        } else {
            double velocity = outtake.getVelocity();
            double error = shootvel - velocity;
            double feedback = error * 0.005;
            double feedforward = 0.00036 * shootvel + 0.08;
            outtake.setPower(feedback + feedforward);
            outtake2.setPower(feedback + feedforward);
        }

        // ====================================================================
        // TURRET MANUAL TRIM
        // ====================================================================
        if (gamepad2.dpadLeftWasPressed() || gamepad1.dpadLeftWasPressed()) {
            turret.adjustTrim(-ServoTurret.MANUAL_TRIM_STEP);
        } else if (gamepad1.dpadRightWasPressed() || gamepad1.dpadRightWasPressed()) {
            turret.adjustTrim(ServoTurret.MANUAL_TRIM_STEP);
        }

        // ====================================================================
        // TURRET HEADING OFFSET  (X = nudge left, B = nudge right, Y = reset)
        // ====================================================================


        // ====================================================================
        // TELEMETRY
        // ====================================================================
        Pose pose = follower.getPose();

        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X", "%.1f in", pose.getX());
        telemetry.addData("Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Loop Time (ms)", "%.2f", loopTime);
        telemetry.addData("servo pos", turret.getPosition());
        telemetry.addData("dis to goal", turret.getDistanceToGoal());
        telemetry.addData("target vel", shootvel);
        telemetry.addData("realvel", outtake.getVelocity());
        telemetry.addData("turret trim", ServoTurret.SERVO_TRIM);
        telemetry.addData("heading offset", "%.1f°", Math.toDegrees(ServoTurret.HEADING_OFFSET));
        telemetry.update();
    }

    @Override
    public void stop() {}
}