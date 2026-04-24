package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Silex", group = "Examples")
public class Silex extends OpMode {

    // MECANISMOS
    private DcMotor Intake;
    private DcMotorEx flywheel, flywheelD;

    // SERVOS
    private Servo cuchara1, cuchara2, cuchara3;
    private Servo Juan;

    // SENSORES COLOR
    private NormalizedColorSensor colorA1, colorA2, colorB1, colorB2, colorC1, colorC2;

    // LIMELIGHT
    private Limelight3A limelight;

    // PIDF
    private static final double TICKS_POR_REVOLUCION = 28.0;
    private static final double TARGET_RPM = 3000.0;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_POR_REVOLUCION) / 60.0;
    private static double F = 32767.0 / 2800.0;
    private static double P = 10.0;
    private static double I = 0.0;
    private static double D = 0.0;

    // RANGOS CUCHARAS
    private static final double C1_MIN = 0.49, C1_MAX = 1.00;
    private static final double C2_MIN = 0.32, C2_MAX = 0.85;
    private static final double C3_MIN = 0.57, C3_MAX = 1.00;

    // RANGOS JUAN
    private static final double JUAN_MIN = 0.20;
    private static final double JUAN_MAX = 1.00;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose = new Pose(70, 85, Math.toRadians(135));
    private final Pose pickup1Pose = new Pose(22, 84, Math.toRadians(197));
    private final Pose pickup2Pose = new Pose(22, 60, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(22, 36, Math.toRadians(185));
    private final Pose out = new Pose(50, 60, Math.toRadians(135));

    private final Pose controlPose1 = new Pose(77, 57, Math.toRadians(180));
    private final Pose controlPose2 = new Pose(85, 35, Math.toRadians(185));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leaveauto;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPose1, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leaveauto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, out))
                .setLinearHeadingInterpolation(scorePose.getHeading(), out.getHeading())
                .build();
    }

    private void resetCucharas() {
        cuchara1.setPosition(C1_MIN);
        cuchara2.setPosition(C2_MIN);
        cuchara3.setPosition(C3_MAX); // invertida: este es su "abajo" en INIT
    }

    private void dispararTresCucharas(double t) {
        // Cuchara 1
        if (t < 2) {
            cuchara1.setPosition(C1_MIN);
        } else if (t < 2.5) {
            cuchara1.setPosition(C1_MAX);
        } else {
            cuchara1.setPosition(C1_MIN);
        }

        // Cuchara 2
        if (t < 3) {
            cuchara2.setPosition(C2_MIN);
        } else if (t < 3.5) {
            cuchara2.setPosition(1);
        } else {
            cuchara2.setPosition(C2_MIN);
        }

        // Cuchara 3 invertida
        if (t < 4) {
            cuchara3.setPosition(C3_MAX);
        } else if (t < 4.5) {
            cuchara3.setPosition(C3_MIN);
        } else {
            cuchara3.setPosition(C3_MAX);
        }
    }

    private void dispararUltimaCuchara(double t) {
        if (t < 1.0) {
            cuchara1.setPosition(C1_MIN);
        } else if (t < 1.3) {
            cuchara1.setPosition(C1_MAX);
        } else {
            cuchara1.setPosition(C1_MIN);
        }
    }

    public void autonomousPathUpdate() {
        System.out.println("trigger");
        switch (pathState) {
            case 0:
                Juan.setPosition(0.4);

                flywheel.setVelocity(1500);
                flywheelD.setVelocity(1500);

                resetCucharas();

                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {

                    double t = pathTimer.getElapsedTimeSeconds();
                    dispararTresCucharas(t);

                    if (t > 5 ) {

                        Intake.setPower(1);
                    }

                    if (t > 7.0) {
                        follower.followPath(grabPickup1, true);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    Intake.setPower(1);

                    double t = pathTimer.getElapsedTimeSeconds();
                    if (t > 1.5) {
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    Intake.setPower(0);

                    double t = pathTimer.getElapsedTimeSeconds();
                    dispararTresCucharas(t);

                    if (t > 5.0) {
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    Intake.setPower(1);

                    double t = pathTimer.getElapsedTimeSeconds();
                    if (t > 1.5) {
                        follower.followPath(scorePickup2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    Intake.setPower(0);

                    double t = pathTimer.getElapsedTimeSeconds();
                    dispararTresCucharas(t);

                    if (t > 5.0) {
                        follower.followPath(grabPickup3, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    Intake.setPower(1);

                    double t = pathTimer.getElapsedTimeSeconds();
                    if (t > 1.5) {
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                    }
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    Intake.setPower(0);

                    double t = pathTimer.getElapsedTimeSeconds();
                    dispararUltimaCuchara(t);

                    if (t > 2.0) {
                        follower.followPath(leaveauto, true);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    resetCucharas();
                    Intake.setPower(0);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("c1 init", C1_MIN);
        telemetry.addData("c2 init", C2_MIN);
        telemetry.addData("c3 init invertida", C3_MAX);
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // MECANISMOS
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        // FLYWHEELS
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheelD = hardwareMap.get(DcMotorEx.class, "FlywheelD");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheelD.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVOS
        cuchara1 = hardwareMap.get(Servo.class, "cuchara1");
        cuchara2 = hardwareMap.get(Servo.class, "cuchara2");
        cuchara3 = hardwareMap.get(Servo.class, "cuchara3");
        Juan = hardwareMap.get(Servo.class, "Juan");

        // Direcciones
        cuchara1.setDirection(Servo.Direction.FORWARD);
        cuchara2.setDirection(Servo.Direction.FORWARD);
        cuchara3.setDirection(Servo.Direction.FORWARD);
        Juan.setDirection(Servo.Direction.FORWARD);

        // Posiciones iniciales al presionar INIT
        cuchara1.setPosition(C1_MIN);
        cuchara2.setPosition(C2_MIN);
        cuchara3.setPosition(C3_MAX); // invertida: queda abajo físicamente
        Juan.setPosition(JUAN_MIN);

        // SENSORES COLOR
        colorA1 = hardwareMap.get(NormalizedColorSensor.class, "colorA1");
        colorA2 = hardwareMap.get(NormalizedColorSensor.class, "colorA2");
        colorB1 = hardwareMap.get(NormalizedColorSensor.class, "colorB1");
        colorB2 = hardwareMap.get(NormalizedColorSensor.class, "colorB2");
        colorC1 = hardwareMap.get(NormalizedColorSensor.class, "colorC1");
        colorC2 = hardwareMap.get(NormalizedColorSensor.class, "colorC2");

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // PIDF
        PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        flywheelD.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // LIMELIGHT
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("INIT activo");
        telemetry.addData("cuchara1", "MIN");
        telemetry.addData("cuchara2", "MIN");
        telemetry.addData("cuchara3", "MAX por REVERSE");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        Intake.setPower(0);
        flywheel.setPower(0);
        flywheelD.setPower(0);
    }
}