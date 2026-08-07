package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret2;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.gate;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Configurable
@Autonomous
public class red2spike21notsolo extends OpMode {

    private ServoTurret turret = new ServoTurret();
    private hood hood = new hood();
    private gate gate = new gate();
    private intake intake = new intake();

    private int count = 0;

    public double vel;

    private Follower follower;
    Timer pathTimer, opModeTimer, shootTimer;

    private DcMotorEx outtake, outtake2;

    private boolean isShooting = false;

    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_SET2,
        INTAKE_SET2,
        DRIVE_SET2_TO_SHOOT,
        SHOOT_SET2,
        DRIVE_TO_GATE,
        DRIVE_GATE_TO_SHOOT,
        ADJUST,
        SHOOT_GATE,
        DRIVE_TO_SET1,
        INTAKE_SET1,
        DRIVE_SET1_TO_SHOOT,
        SHOOT_SET1,
        INTAKE_SET1_THEN_GATE,
        DRIVE_SET1_TO_GATE,
        DRIVE_TO_SET3,
        INTAKE_SET3,
        DRIVE_SET3_TO_SHOOT,
        SHOOT_SET3,
        DRIVE_TO_END,
        IDLE
    }

    PathState pathState;

    private PathChain driveStartToShoot, driveToSet1, driveSet1ToShoot, driveToGate, driveGateToShoot, driveToSet2, driveSet2ToShoot, driveToSet3, driveSet3ToShoot, driveToEnd;

    private final double SHOOT_TIME = 0.5;
    private final double INTAKE_TIME = 0;
    private final double INTAKE_GATE_TIME = 1.5;

    public void buildPaths() {
        driveStartToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(121.450, 120.448),
                                new Pose(90.577, 74.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
                .build();

        driveToSet2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.577, 74.143),
                                new Pose(103.153, 55.795, Math.toRadians(0)),
                                new Pose(131.526, 58.373)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        driveSet2ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.526, 58.373),
                                new Pose(103.153, 55.795),
                                new Pose(90.577, 74.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        driveToGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(90.577, 74.143),
                                new Pose(98, 55),
                                new Pose(134.139, 57.027)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
                .build();

        driveGateToShoot = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(134.139, 57.027),
                                new Pose(98, 55),
                                new Pose(90.577, 74.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .build();

        driveToSet1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.577, 74.143),
                                new Pose(126.56949152542373, 82.94237288135591)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        driveSet1ToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.56949152542373, 82.94237288135591),
                                new Pose(90.577, 74.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(90.577, 74.143),
                                new Pose(92.000, 71.00)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private void updateShooters() {
        double goalDistance = turret.getDistanceToGoal();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                intake.allspin();
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    gate.close();
                    follower.followPath(driveToSet2, true);
                    setPathState(PathState.DRIVE_TO_SET2);
                    intake.allspin();
                }
                break;

            case DRIVE_TO_SET2:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET2);
                }
                break;

            case INTAKE_SET2:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    intake.stop();
                    gate.open();
                    follower.followPath(driveSet2ToShoot, true);
                    setPathState(PathState.DRIVE_SET2_TO_SHOOT);
                }
                break;

            case DRIVE_SET2_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET2);
                }
                break;

            case SHOOT_SET2:
                intake.allspin();

                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    gate.close();
                    intake.intakeonly();
                    follower.followPath(driveToGate, true);
                    setPathState(PathState.DRIVE_TO_GATE);
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy()) {
                    setPathState(PathState.ADJUST);
                }
                break;

            case ADJUST:
                double gateWaitTime;
                switch (count) {
                    case 0: gateWaitTime = 0.8; break;
                    case 1: gateWaitTime = 0.95; break;
                    case 2: gateWaitTime = 0.8; break;
                    case 3: gateWaitTime = 0.95; break;
                    default: gateWaitTime = 1.5; break;
                }

                if (pathTimer.getElapsedTimeSeconds() >= gateWaitTime) {
                    follower.followPath(driveGateToShoot, true);
                    setPathState(PathState.DRIVE_GATE_TO_SHOOT);
                    intake.allspin();
                }
                break;

            case DRIVE_GATE_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.3) {
                    intake.stop();
                } else {
                    intake.allspin();
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_GATE);
                }
                break;

            case SHOOT_GATE:
                gate.open();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    intake.intakeonly();
                    gate.close();
                    count++;

                    switch (count) {
                        case 1:
                            follower.followPath(driveToGate, true);
                            setPathState(PathState.DRIVE_TO_GATE);
                            break;

                        case 2:
                            follower.followPath(driveToSet1, true);
                            setPathState(PathState.INTAKE_SET1_THEN_GATE);
                            break;

                        case 3:
                            follower.followPath(driveToGate, true);
                            setPathState(PathState.DRIVE_TO_GATE);
                            break;

                        case 4:
                            follower.followPath(driveToGate, true);
                            setPathState(PathState.DRIVE_TO_GATE);
                            break;
                        default:
                            follower.followPath(driveToEnd, true);
                            setPathState(PathState.DRIVE_TO_END);
                            break;
                    }
                }
                break;

            case INTAKE_SET1_THEN_GATE:
                if (!follower.isBusy()) {
                    intake.allspin();
                    setPathState(PathState.DRIVE_SET1_TO_GATE);
                }
                break;

            case DRIVE_SET1_TO_GATE:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    intake.allspin();
                    follower.followPath(driveSet1ToShoot, true);
                    setPathState(PathState.DRIVE_GATE_TO_SHOOT);
                }
                break;

            case DRIVE_TO_SET1:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET1);
                }
                break;

            case INTAKE_SET1:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    intake.allspin();
                    follower.followPath(driveSet1ToShoot, true);
                    setPathState(PathState.DRIVE_SET1_TO_SHOOT);
                }
                break;

            case DRIVE_SET1_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET1);
                }
                break;

            case SHOOT_SET1:
                gate.open();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    gate.close();
                    intake.intakeonly();
                    follower.followPath(driveToEnd, true);
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;

            default:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        pathState = PathState.DRIVE_START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        turret.init(hardwareMap);
        hood.init(hardwareMap);
        gate.init(hardwareMap);
        intake.init(hardwareMap);

        buildPaths();
        follower.setPose(new Pose(121.44975288303131, 116.4481054365733, Math.toRadians(0)));

        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
        gate.close();
    }

    public void start() {
        opModeTimer.resetTimer();
        follower.followPath(driveStartToShoot, true);
        setPathState(PathState.DRIVE_START_TO_SHOOT);
        turret.adjustTrim(+0.01);
        gate.open();
    }

    @Override
    public void loop() {
        vel = (int) MathFunctions.clamp(
                -0.00000496881 * Math.pow(turret.getDistanceToGoal(), 4)
                        + 0.00196997 * Math.pow(turret.getDistanceToGoal(), 3)
                        - 0.262396 * Math.pow(turret.getDistanceToGoal(), 2)
                        + 18.24098 * turret.getDistanceToGoal()
                        + 485.32109,
                0,
                1720
        );

        follower.update();
        statePathUpdate();
        turret.update(follower);
        hood.setPosition(hood.autoshoot(turret.getDistanceToGoal())-0.1);

        if (isShooting) {
            updateShooters();
        }

        double velocity = (outtake.getVelocity());
        double error = vel - velocity;
        double feedback = error * 0.005;
        double feedforward = 0.00036 * vel + 0.08;
        outtake.setPower(feedback + feedforward);
        outtake2.setPower(feedback + feedforward);

        telemetry.addData("State", pathState);
        telemetry.addData("Elapsed Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("State Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shoot Timer", shootTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Count", count);
        telemetry.update();
    }

    @Override
    public void stop(){StateTransfer.posePedro = follower.getPose();}
}