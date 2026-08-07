package org.firstinspires.ftc.teamcode.pedroPathing;

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

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret2;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.gate;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Autonomous
public class blindred24 extends OpMode {
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
    PathChain driveToSet1, driveToShootPre, driveSet1ToShoot, driveToLoadingZone, driveToLoadingZoneToShoot, driveToEnd;

    private final double SHOOT_TIME = 0.60;
    private final double INTAKE_TIME = 0;

    public enum PathState {
        SPINUP,
        SHOOT_PRELOAD,
        DRIVE_TO_SET1,
        INTAKE_SET1,
        DRIVE_SET1_TO_SHOOT,
        SHOOT_SET1,
        DRIVE_TO_LOADING_ZONE,
        DRIVE_LOADING_ZONE_TO_SHOOT,
        SHOOT_LOADING_ZONE,
        DRIVE_TO_END,
        IDLE
    }

    PathState pathState;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        turret.init(hardwareMap);
        hood.init(hardwareMap);
        gate.init(hardwareMap);
        intake.init(hardwareMap);

        buildPaths();
        follower.setPose(new Pose(90.5, 8, Math.toRadians(0)));

        gate.close();
        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
    }

    @Override
    public void start() {
        turret.adjustTrim(+0.01);
        opModeTimer.resetTimer();
        gate.close();
        follower.followPath(driveToShootPre, true);
        setPathState(PathState.SPINUP);
    }

    @Override
    public void loop() {
        vel = (int) MathFunctions.clamp(
                -0.00000496881 * Math.pow(turret.getDistanceToGoal(), 4)
                        + 0.00196997 * Math.pow(turret.getDistanceToGoal(), 3)
                        - 0.262396 * Math.pow(turret.getDistanceToGoal(), 2)
                        + 18.24098 * turret.getDistanceToGoal()
                        + 455.32109,
                0,
                1720
        );

        follower.update();
        statePathUpdate();
        turret.update(follower);
        hood.setPosition(hood.autoshoot(turret.getDistanceToGoal()));

        double velocity = outtake.getVelocity();
        double error = vel - velocity;
        double feedback = error * 0.005;
        double feedforward = 0.00036 * vel + 0.08;
        outtake.setPower(feedback + feedforward);
        outtake2.setPower(feedback + feedforward);

        telemetry.addData("State", pathState);
        telemetry.addData("Elapsed Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("State Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Cycle Count", count);
        telemetry.update();
    }

    public void buildPaths() {
        driveToShootPre = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90.500, 8.000),

                        new Pose(93.708, 15.186)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                .build();
        driveToSet1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(93.708, 15.186),
                        new Pose(102.359, 40.545),
                        new Pose(132.292, 35.186)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                .build();

        driveSet1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(128.292, 35.186),
                        new Pose(88.000, 9.500)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        driveToLoadingZone = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(8.000, 9.500),
                        new Pose(137.137, 7.794)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveToLoadingZoneToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(137.137, 7.794),
                        new Pose(100.000, 9.500)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(100.000, 9.500),
                        new Pose(120.562, 10)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case SPINUP:
                intake.allspin();

                setPathState(PathState.SHOOT_PRELOAD);


                // --- Shoot the preloaded rings at starting position ---
            case SHOOT_PRELOAD:

                if (pathTimer.getElapsedTimeSeconds()>2){
                    gate.open();
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.7) {
                    gate.close();
                    intake.intakeonly();
                    follower.followPath(driveToSet1, true);
                    setPathState(PathState.DRIVE_TO_SET1);
                }
                break;

            // --- Drive out to the far ring set ---
            case DRIVE_TO_SET1:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET1);
                }
                break;

            // --- Intake rings at Set1, then drive back to shoot position ---
            case INTAKE_SET1:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    intake.allspin();
                    follower.followPath(driveSet1ToShoot, true);
                    setPathState(PathState.DRIVE_SET1_TO_SHOOT);
                }
                break;

            // --- Drive back from Set1 to shoot position ---
            case DRIVE_SET1_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET1);
                }
                break;

            // --- Shoot rings collected from Set1 ---
            case SHOOT_SET1:
                intake.allspin();
                gate.open();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    gate.close();
                    intake.intakeonly();
                    follower.followPath(driveToLoadingZone, true);
                    setPathState(PathState.DRIVE_TO_LOADING_ZONE);
                }
                break;

            // --- Drive to loading zone corner, start intake, then immediately begin return ---
            case DRIVE_TO_LOADING_ZONE:
                if (!follower.isBusy()) {
                    intake.allspin(); // start picking up rings at the corner
                    follower.followPath(driveToLoadingZoneToShoot, true); // ✅ start return drive
                    setPathState(PathState.DRIVE_LOADING_ZONE_TO_SHOOT);
                }
                break;

            // --- Drive back from loading zone to shoot position while intaking ---
            case DRIVE_LOADING_ZONE_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.3){
                    intake.stop();
                }
                else{
                    intake.allspin();}
                if (!follower.isBusy()) {

                    setPathState(PathState.SHOOT_LOADING_ZONE);
                }
                break;

            // --- Shoot loading zone rings, then cycle again or end ---
            case SHOOT_LOADING_ZONE:
                gate.open();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    gate.close();
                    intake.intakeonly();
                    count++;

                    if (count < 7) {
                        // ✅ Cycle back to loading zone for another pass
                        follower.followPath(driveToLoadingZone, true);
                        setPathState(PathState.DRIVE_TO_LOADING_ZONE);
                    } else {
                        // Done all 3 cycles, park
                        follower.followPath(driveToEnd, true);
                        setPathState(PathState.DRIVE_TO_END);
                    }
                }
                break;

            // --- Drive to parking position ---
            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}