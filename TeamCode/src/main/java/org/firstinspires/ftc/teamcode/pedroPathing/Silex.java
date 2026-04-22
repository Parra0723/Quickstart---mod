package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
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
    private static final double TARGET_RPM = 5000.0;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_POR_REVOLUCION) / 60.0;
    private static double F = 32767.0 / 2800.0;
    private static double P = 10.0;
    private  static double I = 0.0;
    private static double D = 0.0;

    // RANGOS CUCHARAS
    private static final double C1_MIN = 0.49, C1_MAX = 1;
    private static final double C2_MIN = 0.32, C2_MAX = 0.85;
    private static final double C3_MIN = 0.57, C3_MAX = 1.00;

    // RANGOS SERVO JUAN
    private static final double JUAN_MIN = 0.20;
    private static final double JUAN_MAX = 1.00;
    private double juanActualPos = JUAN_MIN;

    private static final double POS_DESIEMPRE = 0.0;
    private static final double POS_ACTIVADA  = 1.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(22, 84, Math.toRadians(197)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(22, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(22, 36, Math.toRadians(185)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private final Pose out = new Pose(50, 60, Math.toRadians(135)); // Salida del triangulo

//    controlpose
    private final Pose controlPose1 = new Pose(77, 57, Math.toRadians(180));
    private final Pose controlpose2 = new Pose(85, 35, Math.toRadians(185));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, leaveauto;

    public void buildPaths() {
        // Shootea las primeras precargadas :)
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        // Se traga la primera fila de pelota :)
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Shootea la primera fila de pelotas
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // Se traga la segunda fila de pelotas
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPose1, pickup2Pose) )
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // Shootea la segunda fila de pelotas
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // Se traga la tercera fila de pelotas
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlpose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        // Shootea la tercera fila de pelotas
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        // Sale del triangulo permitido para shootear
        leaveauto= follower.pathBuilder()
                .addPath(new BezierLine(scorePose, out))
                .setLinearHeadingInterpolation(scorePose.getHeading(), out.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                Juan.setPosition(1);
                // Inicializar shooter
                flywheel.setVelocity(3000);
                flywheelD.setVelocity(3000);

                follower.followPath(scorePreload);
                setPathState(1);


                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 3) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    //Posicion de Juan por siempre y para siempre
                    Juan.setPosition(0.65);

                    // Secuencia de servos cucharas (indexer)
                    if (pathTimer.getElapsedTimeSeconds() > 2) {
                        System.out.println("debug pathTimer.getElapsedTimeSeconds(): "+pathTimer.getElapsedTimeSeconds());
                        System.out.println("debug C1_MAX:"+C1_MAX);
                        cuchara1.setPosition(C1_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() < 2) {
                     cuchara1.setPosition(0.49);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 3 && pathTimer.getElapsedTimeSeconds() < 3.5) {
                        cuchara2.setPosition(C2_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                        cuchara2.setPosition(C2_MIN);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 4) {
                        cuchara3.setPosition(C3_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 4.5) {
                        cuchara3.setPosition(C3_MIN);
                    }

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 6) {
                        follower.followPath(grabPickup2, true);
                        setPathState(2);
                    }


                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {

                    // Agarra pelotas de fila 1
                    Intake.setPower(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(scorePickup1, true);
                        setPathState(3);
                    }

                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    // Shootea primera linea de pelotas.
                    // Intake OFF
                    Intake.setPower(0);

                    // Servos otra vez
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        cuchara1.setPosition(C1_MAX);
                    } else { cuchara1.setPosition(C1_MIN);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 2) {
                        cuchara2.setPosition(C2_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        cuchara2.setPosition(C2_MIN);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        cuchara3.setPosition(C3_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 3) {
                        cuchara3.setPosition(C3_MIN);
                    }

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(grabPickup2, true);
                        setPathState(4);
                    }

                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    // Agarra 2da linea de pelotas
                    // Intake ON otra vez
                    Intake.setPower(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(scorePickup2, true);
                        setPathState(5);
                    }

                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    // Shootea 2da linea de pelotas
                    // Intake OFF
                    Intake.setPower(0);

                    // Servos again
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        cuchara1.setPosition(C1_MAX);
                    } else { cuchara1.setPosition(C1_MIN);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1 && pathTimer.getElapsedTimeSeconds() < 2) {
                        cuchara2.setPosition(C2_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 2) {
                        cuchara2.setPosition(C2_MIN);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        cuchara3.setPosition(C3_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 3) {
                        cuchara3.setPosition(C3_MIN);
                    }


                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(grabPickup3, true);
                        setPathState(6);
                    }

                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    // Agarra 3ra linea de pelotas
                    // Intake ON again
                    Intake.setPower(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(scorePickup3, true);
                        setPathState(7);
                    }

                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    // Shootea 3era linea de pelotas.
                    // Intake OFF
                    Intake.setPower(0);


                    if (pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) {
                        cuchara1.setPosition(C1_MAX);
                    } else if (pathTimer.getElapsedTimeSeconds() > 3) {
                        cuchara1.setPosition(C1_MIN);
                    }

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(leaveauto, true);
                        setPathState(8);
                    }

                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // MECANISMOS
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        // FLYWHEELS
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheelD= hardwareMap.get(DcMotorEx.class, "FlywheelD");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheelD.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVOS
        cuchara1 = hardwareMap.get(Servo.class, "cuchara1");
        cuchara2 = hardwareMap.get(Servo.class, "cuchara2");
        cuchara3 = hardwareMap.get(Servo.class, "cuchara3");
        Juan     = hardwareMap.get(Servo.class, "Juan");

        // SENSORES COLOR
        colorA1 = hardwareMap.get(NormalizedColorSensor.class, "colorA1");
        colorA2 = hardwareMap.get(NormalizedColorSensor.class, "colorA2");
        colorB1 = hardwareMap.get(NormalizedColorSensor.class, "colorB1");
        colorB2 = hardwareMap.get(NormalizedColorSensor.class, "colorB2");
        colorC1 = hardwareMap.get(NormalizedColorSensor.class, "colorC1");
        colorC2 = hardwareMap.get(NormalizedColorSensor.class, "colorC2");

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // =========================== CONFIG PIDF FLYWHEEL ===========================

        PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        flywheelD.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // CONFIG CUCHARAS Y JUAN
        cuchara1.scaleRange(C1_MIN, C1_MAX);
        cuchara2.scaleRange(C2_MIN, C2_MAX);
        cuchara3.scaleRange(C3_MIN, C3_MAX);
        cuchara1.setPosition(POS_DESIEMPRE);
        cuchara2.setPosition(POS_DESIEMPRE);
        cuchara3.setPosition(POS_ACTIVADA);

        Juan.scaleRange(JUAN_MIN, JUAN_MAX);
        Juan.setPosition(juanActualPos);


        // LIMELIGHT
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

    }

}