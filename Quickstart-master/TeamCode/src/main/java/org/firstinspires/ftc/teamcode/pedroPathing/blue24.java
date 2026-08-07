//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.MathFunctions;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret2;
//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.gate;
//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;
//
//@Configurable
//@Autonomous
//public class blue24 extends OpMode {
//
//    // ============================================================================
//    // MECHANISMS
//    // ============================================================================
//
//    private Follower     follower;
//    private ServoTurret2 turret  = new ServoTurret2();
//    private hood         hood    = new hood();
//    private intake       intake  = new intake();
//    private gate         gate    = new gate();
//    private DcMotorEx    outtake, outtake2;
//
//    // ============================================================================
//    // TIMERS
//    // ============================================================================
//
//    private Timer pathTimer   = new Timer();
//    private Timer shootTimer  = new Timer();
//    private Timer opModeTimer = new Timer();
//
//    // ============================================================================
//    // TUNABLE CONSTANTS
//    // ============================================================================
//
//    public static double SCORE_WAIT_S      = 1.0;
//    public static double INTAKE_WAIT_S     = 2.0;
//    public static double GATE_OPEN_DELAY_S = 0.1;
//
//    // ============================================================================
//    // PATHS
//    // ============================================================================
//
//    private Paths paths;
//
//    public static class Paths {
//
//        // ── Key poses ────────────────────────────────────────────────────────
//        private static final Pose START    = new Pose(19.650, 130.448);
//        private static final Pose SCORE    = new Pose(55.604,  72.044);
//        private static final Pose SAMPLE   = new Pose(10.814,  58.649);
//        private static final Pose PARK_POS = new Pose(23.465,  37.000);
//
//        // ── Headings ─────────────────────────────────────────────────────────
//        private static final double H_FORWARD   = Math.toRadians(180);
//        private static final double H_SCORE_OUT = Math.toRadians(190);
//        private static final double H_SWEEP     = Math.toRadians(150);
//        private static final double H_WIDE      = Math.toRadians(200);
//        private static final double H_TURN      = Math.toRadians(250);
//        private static final double H_PARK      = Math.toRadians(270);
//        private static final double H_PARK_END  = Math.toRadians(260);
//
//        // ── Individual path chains ────────────────────────────────────────────
//        public PathChain Seg0_PreloadDrive;  // START → SCORE
//        public PathChain Seg1_SweepArc;      // SCORE → sweep → SCORE
//        public PathChain Seg2_ToSample1;     // SCORE → SAMPLE 1
//        public PathChain Seg3_ToScore1;      // SAMPLE 1 → SCORE
//        public PathChain Seg4_ToSample2;     // SCORE → SAMPLE 2
//        public PathChain Seg5_ToScore2A;     // SAMPLE 2 → curve up
//        public PathChain Seg6_ToScore2B;     // curve up → SCORE (wide arc)
//        public PathChain Seg7_ToSample3;     // SCORE → SAMPLE 3
//        public PathChain Seg8_ToScore3;      // SAMPLE 3 → SCORE curved
//        public PathChain Seg9_ToPark;        // SCORE → PARK zone
//        public PathChain Seg10_ToFinal;      // PARK → final score position
//        public PathChain Seg11_Settle;       // settle into final position
//
//        public Paths(Follower follower) {
//
//            Seg0_PreloadDrive = follower.pathBuilder()
//                    .addPath(new BezierLine(START, new Pose(55.202, 72.042)))
//                    .setLinearHeadingInterpolation(H_FORWARD, H_SCORE_OUT)
//                    .build();
//
//            Seg1_SweepArc = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(55.202, 72.042),
//                            new Pose(14.898, 52.009),
//                            new Pose(12.607, 61.265),
//                            SCORE))
//                    .setLinearHeadingInterpolation(H_SCORE_OUT, H_FORWARD)
//                    .build();
//
//            Seg2_ToSample1 = follower.pathBuilder()
//                    .addPath(new BezierLine(SCORE, new Pose(10.761, 58.727)))
//                    .setLinearHeadingInterpolation(H_FORWARD, H_SWEEP)
//                    .build();
//
//            Seg3_ToScore1 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(10.761, 58.727), new Pose(55.604, 71.044)))
//                    .setLinearHeadingInterpolation(H_SWEEP, H_FORWARD)
//                    .build();
//
//            Seg4_ToSample2 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(55.604, 71.044), SAMPLE))
//                    .setLinearHeadingInterpolation(H_FORWARD, H_SWEEP)
//                    .build();
//
//            Seg5_ToScore2A = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            SAMPLE,
//                            new Pose(45.252, 63.429),
//                            new Pose(43.943, 82.490)))
//                    .setLinearHeadingInterpolation(H_SWEEP, H_FORWARD)
//                    .build();
//
//            Seg6_ToScore2B = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(43.943, 82.490),
//                            new Pose(-5.351, 86.087),
//                            new Pose(55.912, 75.067)))
//                    .setLinearHeadingInterpolation(H_FORWARD, H_WIDE)
//                    .build();
//
//            Seg7_ToSample3 = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(55.912, 75.067), new Pose(10.906, 58.723)))
//                    .setLinearHeadingInterpolation(H_WIDE, H_SWEEP)
//                    .build();
//
//            Seg8_ToScore3 = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(10.906, 58.723),
//                            new Pose(37.641, 65.484),
//                            new Pose(56.376, 74.244)))
//                    .setLinearHeadingInterpolation(H_SWEEP, H_TURN)
//                    .build();
//
//            Seg9_ToPark = follower.pathBuilder()
//                    .addPath(new BezierCurve(
//                            new Pose(56.376, 74.244),
//                            new Pose(58.271, 69.686),
//                            new Pose(43.376, 36.846),
//                            PARK_POS))
//                    .setLinearHeadingInterpolation(H_TURN, H_FORWARD)
//                    .build();
//
//            Seg10_ToFinal = follower.pathBuilder()
//                    .addPath(new BezierLine(PARK_POS, new Pose(58.251, 66.053)))
//                    .setLinearHeadingInterpolation(H_FORWARD, H_PARK)
//                    .build();
//
//            Seg11_Settle = follower.pathBuilder()
//                    .addPath(new BezierLine(new Pose(58.251, 66.053), new Pose(57.852, 65.950)))
//                    .setLinearHeadingInterpolation(H_PARK, H_PARK_END)
//                    .build();
//        }
//    }
//
//    // ============================================================================
//    // STATE MACHINE
//    // ============================================================================
//
//    public enum PathState {
//        // Preload
//        PRELOAD_DRIVE,
//        PRELOAD_SHOOT,
//        TEMPSHOOT,
//
//        // Cycle 1
//        SWEEP_TO_SAMPLE_1,
//        DRIVE_TO_SAMPLE_1,
//        INTAKE_SAMPLE_1,
//        DRIVE_TO_SCORE_1,
//        SHOOT_1,
//
//        // Cycle 2
//        DRIVE_TO_SAMPLE_2,
//        INTAKE_SAMPLE_2,
//        DRIVE_TO_SCORE_2A,
//        DRIVE_TO_SCORE_2B,
//        SHOOT_2,
//
//        // Cycle 3
//        DRIVE_TO_SAMPLE_3,
//        INTAKE_SAMPLE_3,
//        DRIVE_TO_SCORE_3,
//        SHOOT_3,
//
//        // End
//        DRIVE_TO_PARK,
//        DRIVE_TO_FINAL,
//        SETTLE,
//        IDLE
//    }
//
//    private PathState pathState;
//
//    // ============================================================================
//    // MECHANISM HELPERS
//    // ============================================================================
//
//    private double calcVel() {
//        return MathFunctions.clamp(
//                -0.00000496881 * Math.pow(turret.getDistanceToGoal(), 4)
//                        +       0.00196997 * Math.pow(turret.getDistanceToGoal(), 3)
//                        -         0.262396 * Math.pow(turret.getDistanceToGoal(), 2)
//                        +          18.24098 * turret.getDistanceToGoal()
//                        +          455.32109,
//                0, 1720
//        );
//    }
//
//    private void updateFlywheel(double targetVel) {
//        double velocity    = outtake.getVelocity();
//        double error       = targetVel - velocity;
//        double feedback    = error * 0.005;
//        double feedforward = 0.00036 * targetVel + 0.08;
//        outtake.setPower(feedback + feedforward);
//        outtake2.setPower(feedback + feedforward);
//    }
//
//    private void stopFlywheel() {
//        outtake.setPower(0);
//        outtake2.setPower(0);
//    }
//
//    private void beginShot() {
//        hood.setPosition(hood.autoshoot(turret.getDistanceToGoal()));
//        gate.close();
//        intake.allspin();
//        shootTimer.resetTimer();
//    }
//
//    private void tickShot() {
//        updateFlywheel(calcVel());
//        intake.allspin();
//        if (shootTimer.getElapsedTimeSeconds() >= GATE_OPEN_DELAY_S) {
//            gate.open();
//        }
//    }
//
//    private void endShot() {
//        intake.stop();
//        gate.close();
//        stopFlywheel();
//    }
//
//    // ============================================================================
//    // STATE UPDATE
//    // ============================================================================
//
//    public void autonomousPathUpdate() {
//        turret.update(follower);
//
//        switch (pathState) {
//
//            // ── Preload ───────────────────────────────────────────────────────
//
//            case PRELOAD_DRIVE:
//                updateFlywheel(calcVel());
//                if (!follower.isBusy()) {
//                    beginShot();
//                    setPathState(PathState.PRELOAD_SHOOT);
//                }
//                break;
//
//            case PRELOAD_SHOOT:
//                tickShot();
//                if (pathTimer.getElapsedTimeSeconds() > SCORE_WAIT_S) {
//                    endShot();
//                    intake.intakeonly();
//                    follower.followPath(paths.Seg1_SweepArc, true);
//                    setPathState(PathState.TEMPSHOOT);
//                }
//                break;
//
//            case TEMPSHOOT:
//                tickShot();
//                if (!follower.isBusy()) {
//                    setPathState(PathState.SWEEP_TO_SAMPLE_1);
//                }
//                break;
//
//            // ── Cycle 1 ───────────────────────────────────────────────────────
//
//            case SWEEP_TO_SAMPLE_1:
//                tickShot();
//                if (pathTimer.getElapsedTimeSeconds() > SCORE_WAIT_S) {
//                    endShot();
//                    intake.intakeonly();
//                    follower.followPath(paths.drive, true);
//                    setPathState(PathState.TEMPSHOOT);
//                }
//                break;
//
//            case DRIVE_TO_SAMPLE_1:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKE_SAMPLE_1);
//                }
//                break;
//
//            case INTAKE_SAMPLE_1:
//                if (pathTimer.getElapsedTimeSeconds() > INTAKE_WAIT_S) {
//                    intake.stop();
//                    updateFlywheel(calcVel());
//                    follower.followPath(paths.Seg3_ToScore1, true);
//                    setPathState(PathState.DRIVE_TO_SCORE_1);
//                }
//                break;
//
//            case DRIVE_TO_SCORE_1:
//                updateFlywheel(calcVel());
//                if (!follower.isBusy()) {
//                    beginShot();
//                    setPathState(PathState.SHOOT_1);
//                }
//                break;
//
//            case SHOOT_1:
//                tickShot();
//                if (pathTimer.getElapsedTimeSeconds() > SCORE_WAIT_S) {
//                    endShot();
//                    intake.intakeonly();
//                    follower.followPath(paths.Seg4_ToSample2, true);
//                    setPathState(PathState.DRIVE_TO_SAMPLE_2);
//                }
//                break;
//
//            // ── Cycle 2 ───────────────────────────────────────────────────────
//
//            case DRIVE_TO_SAMPLE_2:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKE_SAMPLE_2);
//                }
//                break;
//
//            case INTAKE_SAMPLE_2:
//                if (pathTimer.getElapsedTimeSeconds() > INTAKE_WAIT_S) {
//                    intake.stop();
//                    updateFlywheel(calcVel());
//                    follower.followPath(paths.Seg5_ToScore2A, true);
//                    setPathState(PathState.DRIVE_TO_SCORE_2A);
//                }
//                break;
//
//            case DRIVE_TO_SCORE_2A:
//                updateFlywheel(calcVel());
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Seg6_ToScore2B, true);
//                    setPathState(PathState.DRIVE_TO_SCORE_2B);
//                }
//                break;
//
//            case DRIVE_TO_SCORE_2B:
//                updateFlywheel(calcVel());
//                if (!follower.isBusy()) {
//                    beginShot();
//                    setPathState(PathState.SHOOT_2);
//                }
//                break;
//
//            case SHOOT_2:
//                tickShot();
//                if (pathTimer.getElapsedTimeSeconds() > SCORE_WAIT_S) {
//                    endShot();
//                    intake.intakeonly();
//                    follower.followPath(paths.Seg7_ToSample3, true);
//                    setPathState(PathState.DRIVE_TO_SAMPLE_3);
//                }
//                break;
//
//            // ── Cycle 3 ───────────────────────────────────────────────────────
//
//            case DRIVE_TO_SAMPLE_3:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.INTAKE_SAMPLE_3);
//                }
//                break;
//
//            case INTAKE_SAMPLE_3:
//                if (pathTimer.getElapsedTimeSeconds() > INTAKE_WAIT_S) {
//                    intake.stop();
//                    updateFlywheel(calcVel());
//                    follower.followPath(paths.Seg8_ToScore3, true);
//                    setPathState(PathState.DRIVE_TO_SCORE_3);
//                }
//                break;
//
//            case DRIVE_TO_SCORE_3:
//                updateFlywheel(calcVel());
//                if (!follower.isBusy()) {
//                    beginShot();
//                    setPathState(PathState.SHOOT_3);
//                }
//                break;
//
//            case SHOOT_3:
//                tickShot();
//                if (pathTimer.getElapsedTimeSeconds() > SCORE_WAIT_S) {
//                    endShot();
//                    follower.followPath(paths.Seg9_ToPark, true);
//                    setPathState(PathState.DRIVE_TO_PARK);
//                }
//                break;
//
//            // ── End ───────────────────────────────────────────────────────────
//
//            case DRIVE_TO_PARK:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Seg10_ToFinal, true);
//                    setPathState(PathState.DRIVE_TO_FINAL);
//                }
//                break;
//
//            case DRIVE_TO_FINAL:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Seg11_Settle, true);
//                    setPathState(PathState.SETTLE);
//                }
//                break;
//
//            case SETTLE:
//                if (!follower.isBusy()) {
//                    setPathState(PathState.IDLE);
//                }
//                break;
//
//            case IDLE:
//                intake.stop();
//                stopFlywheel();
//                break;
//        }
//    }
//
//    private void setPathState(PathState newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//    }
//
//    // ============================================================================
//    // OPMODE LIFECYCLE
//    // ============================================================================
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(19.650, 130.448, Math.toRadians(180)));
//
//        turret.init(hardwareMap);
//        hood.init(hardwareMap);
//        intake.init(hardwareMap);
//        gate.init(hardwareMap);
//
//        outtake = hardwareMap.get(DcMotorEx.class, "o1");
//        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
//        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");
//        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        paths = new Paths(follower);
//        pathState = PathState.PRELOAD_DRIVE;
//
//        telemetry.addLine("Initialized — Ready!");
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        opModeTimer.resetTimer();
//        pathTimer.resetTimer();
//        follower.followPath(paths.Seg0_PreloadDrive, true);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("State",            pathState);
//        telemetry.addData("Op Time",          opModeTimer.getElapsedTimeSeconds());
//        telemetry.addData("State Time",       pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("Shoot Time",       shootTimer.getElapsedTimeSeconds());
//        telemetry.addData("X",                follower.getPose().getX());
//        telemetry.addData("Y",                follower.getPose().getY());
//        telemetry.addData("Heading (deg)",    Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Distance to Goal", turret.getDistanceToGoal());
//        telemetry.addData("Turret Position",  turret.getPosition());
//        telemetry.addData("Flywheel Vel",     outtake.getVelocity());
//        telemetry.addData("Target Vel",       calcVel());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        intake.stop();
//        stopFlywheel();
//    }
//}