package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class DriveTest extends OpMode {


    protected Command updateShooter;
    private int gateCycleNum = -1;
    private DcMotorEx outtake, outtake2;

    Pose startPose = new Pose(17.735, 110.63, Math.toRadians(180));

    protected Pose shootingPose = new Pose(58, 71, Math.toRadians(180));
    protected Pose middlePickupPose = new Pose(13.990, 55, Math.toRadians(180));
    protected Pose middlePickupControlPoint2 = new Pose(42, 57);
    protected Pose closePickupPose = new Pose(20.590, 81.860, Math.toRadians(180));
    protected Pose gateClearControlPoint = new Pose(56.090, 59.210);
    protected Pose gateClearPose = new Pose(19.847, 60.5, Math.toRadians(180));
    protected Pose gatePickupControlPoint = new Pose(20.590, 55.260);

    private double xOffset = -0.4;
    private double yOffset = 1;
    protected Pose[] gatePickupPoses = {
            new Pose(13.3 + xOffset, 56 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.25 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.5 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 56.75 + yOffset, Math.toRadians(150)),
            new Pose(13.3 + xOffset, 57 + yOffset, Math.toRadians(150)),
    };
    protected Pose farPickupPose = new Pose(11.590, 33.210, Math.toRadians(180));
    protected Pose farPickupControlPoint = new Pose(45, 34);
    protected Pose cornerPose = new Pose(13.990, 17.860, Math.toRadians(210));
    protected Pose cornerBackupPose = new Pose(11.690, 8.360, Math.toRadians(180));
    protected Pose farShootingPose = new Pose(51.247, 10.099);
    protected Pose parkPose = new Pose(45.747, 15.099);
    protected Pose closeParkPose = new Pose(56.990, 102.860, Math.toRadians(180));
//    protected Pose goalPose = Constants.BLUE_GOAL_POSE;
    private Follower follower;

    public double vel;
    protected PathChain shootPreloads;
    protected PathChain pickupMiddle;
    protected PathChain shootMiddle;
    protected PathChain clearGate;
    protected PathChain[] pickupGates;
    protected PathChain[] shootGates;
    protected PathChain shootGateAndPark;
    // PathChain shootGate2;
    protected PathChain pickupClose;
    protected PathChain shootClose;
    protected PathChain shootCloseAndPark;
    protected PathChain pickupFar;
    protected PathChain shootFar;
    protected PathChain shootFarAndPark;
    protected PathChain pickupCorner;
    protected PathChain backupCorner;
    protected PathChain shootCorner;
    protected PathChain park;
    protected PathChain shootCornerClose;

    protected void createAutoCommands() {
//        updateShooter = robot.updateShootingSubsystems();

        double shootTime = 150;

        schedule(updateShooter,
                sequential(
//                        shootPreloads(),
                        race(
//                                waitUntil(() -> robot.isShooterReady()),
                                waitMs(500)
                        ),
                        runCycle(pickupMiddle, shootMiddle, shootTime, 700, 600),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupClose, shootClose, shootTime, 900, 500),
                        gateCycle(shootTime, 1000),
                        gateCycle(shootTime, 1500),
                        runCycle(pickupFar, shootFarAndPark, shootTime + 125, 700, 750),
//                        shootAndSetIntaking(),
                        waitMs(500)
//                        robot.setIntakePower(0),
//                        robot.deactivateShooter()
                )
        );
    }


//    protected Command shootPreloads() {
//        return sequential(startFlywheel(), follow(robot.getFollower(), shootPreloads));
//    }

    protected Command runCycle(PathChain pickupPath, PathChain shootPath, double shootDelayMs,
                               double intakeDelayMs, double shootingDelayMs) {
        return sequential(
                parallel(
                        sequential(
                                waitMs(shootDelayMs),
                                parallel(
                                        follow(follower, pickupPath),
                                        sequential(
                                                waitMs(intakeDelayMs)
//                                                shootFar.setIntakePower(1)
                                        )
                                )//.raceWith(waitUntil(() -> shootFar.beamBroken()))
                        )
                ),
                parallel(
                        sequential(
                                waitMs(200)
//                                conditional(
//                                        //() -> shootFar.beamBroken(),
//                                        instant(() -> {}), // do nothing
//                                        sequential(
////                                                shootFar.setIntakePower(-1),
//                                                waitMs(50),
////                                                shootFar.setIntakePower(0)
//                                        )
//                                )
                        ),
                        follow(follower, shootPath),
                        sequential(
                                waitMs(shootingDelayMs)
//                                shootFar.setIntakePower(0)
                        )
                )
        );
    }

    protected Command gateCycle(double shootDelayMs, double gateWaitMs) {
        gateCycleNum++;
        return sequential(
                parallel(
                        sequential(waitMs(shootDelayMs),
                                follow(follower, pickupGates[gateCycleNum])
                        )
                ),
                race(
                        waitMs(gateWaitMs)
//                        waitUntil(() -> robot.beamBroken()) //leave gate early if we have all balls
                ),
                parallel(
                        sequential(
                                waitMs(200)
//                                conditional(
//                                        () -> robot.beamBroken(),
//                                        instant(() -> {}), // do nothing
//                                        sequential(
//                                                waitMs(50)
//                                        )
//                                )
                        ),
                        follow(follower, shootGates[gateCycleNum]),
                        sequential(
                                waitMs(200)
                        )
                )
        );
    }

//    protected Command gateCycleAndPark(double shootDelayMs, double gateWaitMs) {
//        gateCycleNum++;
//        return sequential(
//                        sequential(waitMs(shootDelayMs), robot.setIntakePower(1),
//                                follow(follower, pickupGates[gateCycleNum]))
//                waitMs(gateWaitMs), parallel(follow(follower, shootGateAndPark)
//                        sequential(waitMs(1000), robot.setIntakePower(0))));
//    }



    public static Command turnTo(Follower follower, double radians) {
        return new CommandBuilder().setStart(() -> {
            Pose pose = follower.getPose();
            Path path = new Path(new BezierPoint(pose));
            path.setHeadingInterpolation(HeadingInterpolator.constant(radians));
            follower.followPath(path);
        }).setDone(() -> !follower.isBusy());
    }

//    protected Command shootAndSetIntaking() {
//        return instant(() -> robot.setState(States.SHOOTING));
//    }
//
//    protected Command startFlywheel() {
//        return instant(() -> robot.activateShooter());
//    }

    private void generatePaths() {
        shootPreloads = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupMiddle = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, middlePickupControlPoint2, middlePickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootMiddle = follower.pathBuilder()
                .addPath(new BezierLine(middlePickupPose, shootingPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        for (int i = 0; i < gatePickupPoses.length; i++) {
            if (i == 0) {
                pickupGates = new PathChain[gatePickupPoses.length];
                shootGates = new PathChain[gatePickupPoses.length];
            }
            pickupGates[i] = follower.pathBuilder()
                    .addPath(new BezierLine(shootingPose, gatePickupPoses[i]))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.6, HeadingInterpolator.tangent),
                            new HeadingInterpolator.PiecewiseNode(0.6, 1, HeadingInterpolator.constant(gatePickupPoses[i].getHeading()))
                    ))
                    .build();

            shootGates[i] = follower.pathBuilder()
                    .addPath(new BezierLine(gatePickupPoses[i], shootingPose))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }



        shootGateAndPark = follower.pathBuilder()
                .addPath(new BezierLine(gatePickupPoses[gatePickupPoses.length-1], closeParkPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupClose = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, closePickupPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootClose = follower.pathBuilder()
                .addPath(new BezierLine(closePickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootCloseAndPark = follower.pathBuilder()
                .addPath(new BezierLine(closePickupPose, closeParkPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        pickupFar = follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, farPickupControlPoint, farPickupPose))
//                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();
                .setTangentHeadingInterpolation().build();

        shootFar = follower.pathBuilder()
                .addPath(new BezierLine(farPickupPose, shootingPose))
                .setConstantHeadingInterpolation(shootingPose.getHeading()).build();

        shootFarAndPark = follower.pathBuilder()
                .addPath(new BezierLine(farPickupPose, closeParkPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pickupCorner =
                follower.pathBuilder().addPath(new BezierLine(shootingPose, cornerPose))
                        .setConstantHeadingInterpolation(cornerPose.getHeading()).build();

        backupCorner = follower.pathBuilder()
                .addPath(new BezierLine(cornerPose, cornerBackupPose))
                .setLinearHeadingInterpolation(cornerPose.getHeading(),
                        cornerBackupPose.getHeading())
                .setConstraints(new PathConstraints(0.8, 2, 2, 0.03, 50, 1, 10, 1)).build();

        shootCorner = follower.pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, farShootingPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();

        park = follower.pathBuilder().addPath(new BezierLine(farShootingPose, parkPose))
                .setTangentHeadingInterpolation().build();

        shootCornerClose = follower.pathBuilder()
                .addPath(new BezierLine(cornerBackupPose, closeParkPose))
                .setConstantHeadingInterpolation(cornerBackupPose.getHeading()).build();
    }


    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
        follower = Constants.createFollower(hardwareMap);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        follower.setPose(new Pose(22.55024711696869, 116.4481054365733, Math.toRadians(180)));
        Scheduler.reset();
        generatePaths();
        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
    }

    public void start() {
        createAutoCommands();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        double velocity = (outtake.getVelocity());
        double error = vel - velocity;
        double feedback = error * 0.005;
        double feedforward = 0.00036 * vel + 0.08;
        outtake.setPower(feedback + feedforward);
        outtake2.setPower(feedback + feedforward);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop(){StateTransfer.posePedro = follower.getPose();}
}