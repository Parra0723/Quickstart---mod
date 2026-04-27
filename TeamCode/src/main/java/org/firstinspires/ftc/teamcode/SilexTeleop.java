package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "ORION2026", group = "ORION")
public class SilexTeleop extends LinearOpMode {

    // DRIVE (Ahora usamos DcMotorEx para poder leer la velocidad real)
    private DcMotorEx MotorDD, MotorTD, MotorDI, MotorTI;

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

    // ESTADO GENERAL
    private boolean flywheelOn = false;
    private boolean flywheelButtonPrev = false;
    private int aprilTagId = -1;
    private static final double DEADBAND = 0.05;

    private enum PelotaColor { NINGUNO, MORADO, VERDE }

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

    // HSV
    private static final float MIN_SAT = 0.30f;
    private static final float MIN_VAL = 0.10f;
    private static final float MORADO_H_MIN = 220f;
    private static final float MORADO_H_MAX = 240f;
    private static final float VERDE_H_MIN  = 90f;
    private static final float VERDE_H_MAX  = 150f;

    // AUTO SECUENCIA
    private boolean autoFireActivo = false;
    private double autoFireTimer = 0;
    private int[] secuenciaAuto = {1, 2, 3};
    private boolean dpadUpPrev = false;

    // AUTO-APUNTE LIMELIGHT (BOTÓN X)
    private boolean autoAimActive = false;
    private boolean autoAimButtonPrev = false;
    private static final double AIM_KP = 0.03;
    private static final double AIM_DEADBAND_DEG = 1.0;

    // ===================== CONFIGURACIÓN PIDF FLYWHEEL =====================
    private static final double TICKS_POR_REVOLUCION = 28.0;
    private static final double TARGET_RPM = 5000.0;
    private static final double TARGET_TICKS_PER_SEC = (TARGET_RPM * TICKS_POR_REVOLUCION) / 60.0;

    private static double F = 32767.0 / 2800.0;
    private static double P = 10.0;
    private  static double I = 0.0;
    private static double D = 0.0;

    // ===================== CONVERSIÓN TICKS -> RPM PARA CHASSIS =====================
    // Asumiendo motores tipo goBILDA 312 RPM (ajusta a los tuyos si son diferentes)
    private static final double DRIVE_TICKS_PER_REV = 537.6;
    private static final double DRIVE_TICKS_TO_RPM = 60.0 / DRIVE_TICKS_PER_REV;

    @Override
    public void runOpMode() {

        // MOTORES DRIVE
        MotorDD = hardwareMap.get(DcMotorEx.class, "MotorDD");
        MotorTD = hardwareMap.get(DcMotorEx.class, "MotorTD");
        MotorDI = hardwareMap.get(DcMotorEx.class, "Motor DI");
        MotorTI = hardwareMap.get(DcMotorEx.class, "MotorTI");

        // MECANISMOS
        Intake   = hardwareMap.get(DcMotor.class, "Intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheelD= hardwareMap.get(DcMotorEx.class, "FlywheelD");

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

        // CONFIG DRIVE
        MotorDD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorTD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorTI.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorDI.setDirection(DcMotor.Direction.FORWARD);
        MotorTI.setDirection(DcMotor.Direction.FORWARD);
        MotorDD.setDirection(DcMotor.Direction.REVERSE);
        MotorTD.setDirection(DcMotor.Direction.REVERSE);

        // INTAKE
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        // =========================== CONFIG PIDF FLYWHEEL ===========================
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheelD.setDirection(DcMotor.Direction.REVERSE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        flywheelD.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        // CONFIG CUCHARAS Y JUAN
        cuchara1.scaleRange(C1_MIN, C1_MAX);
        cuchara2.scaleRange(C2_MIN, C2_MAX);
        cuchara3.scaleRange(C3_MIN, C3_MAX);
        cuchara1.setPosition(POS_DESIEMPRE);
        cuchara2.setPosition(POS_DESIEMPRE);
        cuchara3.setPosition(POS_DESIEMPRE);

        Juan.scaleRange(JUAN_MIN, JUAN_MAX);
        Juan.setPosition(juanActualPos);

        // LIMELIGHT
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("ORION2026 FULL SYSTEM LISTO");
        telemetry.addLine("- X: Auto-Apuntar Tag 20/24");
        telemetry.addLine("- B: Juan Posición 0.40");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================== LIMELIGHT ===========================
            double tx = 0.0;
            boolean tagObjetivoVisible = false;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (tags != null && !tags.isEmpty()) {
                    // 1. Memorizar IDs 21, 22 o 23 para la secuencia de pelotas
                    int detectado = (int) tags.get(0).getFiducialId();
                    if (detectado >= 21 && detectado <= 23) {
                        aprilTagId = detectado;
                    }

                    // 2. Buscar si vemos el ID 20 o el 24 para apuntar
                    for (LLResultTypes.FiducialResult f : tags) {
                        int idVisto = (int) f.getFiducialId();

                        if (idVisto == 20 || idVisto == 24) {
                            tagObjetivoVisible = true;
                            tx = f.getTargetXDegrees();
                            break;
                        }
                    }
                }
            }

            // =========================== TOGGLES ===========================
            // Auto-Apunte (Botón X)
            boolean autoAimButton = gamepad2.right_bumper;
            if (autoAimButton && !autoAimButtonPrev) {
                autoAimActive = !autoAimActive;
            }
            autoAimButtonPrev = autoAimButton;

            // =========================== DRIVE (Gamepad 2) ===========================
            double forward = -gamepad2.left_stick_y;
            double strafe  =  gamepad2.left_stick_x;
            double turn    =  gamepad2.right_stick_x;

            if (Math.abs(forward) < DEADBAND) forward = 0;
            if (Math.abs(strafe)  < DEADBAND) strafe  = 0;
            if (Math.abs(turn)    < DEADBAND) turn    = 0;

            // AUTO-APUNTE OVERRIDE
            if (autoAimActive && tagObjetivoVisible) {
                double error = tx;
                if (Math.abs(error) > AIM_DEADBAND_DEG) {
                    turn = error * AIM_KP;
                    turn = Math.max(-0.5, Math.min(0.5, turn));
                } else {
                    turn = 0;
                }
            }

            double pDI = forward + strafe + turn;
            double pTI = forward - strafe + turn;
            double pDD = forward - strafe - turn;
            double pTD = forward + strafe - turn;

            double max = Math.max(Math.max(Math.abs(pDI), Math.abs(pTI)),
                    Math.max(Math.abs(pDD), Math.abs(pTD)));
            if (max > 1.0) { pDI /= max; pTI /= max; pDD /= max; pTD /= max; }

            MotorDI.setPower(pDI);
            MotorTI.setPower(pTI);
            MotorDD.setPower(pDD);
            MotorTD.setPower(pTD);

            // =========================== SENSORES COLOR ===========================
            PelotaColor hueco1 = detectarPelota(colorA1, colorA2);
            PelotaColor hueco2 = detectarPelota(colorB1, colorB2);
            PelotaColor hueco3 = detectarPelota(colorC1, colorC2);
            PelotaColor[] huecos = {hueco1, hueco2, hueco3};

            // =========================== AUTO SECUENCIA ===========================
            boolean dpadUp = gamepad1.ps;
            if (dpadUp && !dpadUpPrev) {
                if (!autoFireActivo) {
                    autoFireActivo = true;
                    autoFireTimer = getRuntime();

                    PelotaColor[] target = {PelotaColor.NINGUNO, PelotaColor.NINGUNO, PelotaColor.NINGUNO};
                    if (aprilTagId == 21)
                        target = new PelotaColor[]{PelotaColor.VERDE, PelotaColor.MORADO, PelotaColor.MORADO};
                    else if (aprilTagId == 22)
                        target = new PelotaColor[]{PelotaColor.MORADO, PelotaColor.VERDE, PelotaColor.MORADO};
                    else if (aprilTagId == 23)
                        target = new PelotaColor[]{PelotaColor.MORADO, PelotaColor.MORADO, PelotaColor.VERDE};

                    boolean[] used = new boolean[3];
                    for (int i = 0; i < 3; i++) {
                        boolean found = false;
                        if (target[i] != PelotaColor.NINGUNO) {
                            for (int h = 0; h < 3; h++) {
                                if (!used[h] && huecos[h] == target[i]) {
                                    secuenciaAuto[i] = h + 1;
                                    used[h] = true;
                                    found = true;
                                    break;
                                }
                            }
                        }
                        if (!found) {
                            for (int h = 0; h < 3; h++) {
                                if (!used[h]) {
                                    secuenciaAuto[i] = h + 1;
                                    used[h] = true;
                                    break;
                                }
                            }
                        }
                    }
                } else {
                    autoFireActivo = false;
                }
            }
            dpadUpPrev = dpadUp;

            boolean actC1 = false, actC2 = false, actC3 = false;

            if (autoFireActivo) {
                double elapsed = getRuntime() - autoFireTimer;
                if (elapsed < 0.5) {
                    if (secuenciaAuto[0] == 1) actC1 = true;
                    if (secuenciaAuto[0] == 2) actC2 = true;
                    if (secuenciaAuto[0] == 3) actC3 = true;
                } else if (elapsed < 1.0) {
                    if (secuenciaAuto[1] == 1) actC1 = true;
                    if (secuenciaAuto[1] == 2) actC2 = true;
                    if (secuenciaAuto[1] == 3) actC3 = true;
                } else if (elapsed < 1.5) {
                    if (secuenciaAuto[2] == 1) actC1 = true;
                    if (secuenciaAuto[2] == 2) actC2 = true;
                    if (secuenciaAuto[2] == 3) actC3 = true;
                } else {
                    autoFireActivo = false;
                }
            } else {
                actC1 = gamepad1.b;
                actC2 = gamepad1.a;
                actC3 = gamepad1.x;
            }

            cuchara1.setPosition(actC1 ? POS_ACTIVADA : POS_DESIEMPRE);
            cuchara2.setPosition(actC2 ? POS_ACTIVADA : POS_DESIEMPRE);
            cuchara3.setPosition(actC3 ? POS_DESIEMPRE : POS_ACTIVADA);

            // =========================== FLYWHEEL PIDF & INTAKE ===========================
            boolean flywheelBtn = gamepad1.right_bumper;
            if (flywheelBtn && !flywheelButtonPrev) flywheelOn = !flywheelOn;
            flywheelButtonPrev = flywheelBtn;

            if (flywheelOn) {
                flywheel.setVelocity(TARGET_TICKS_PER_SEC);
                flywheelD.setVelocity(TARGET_TICKS_PER_SEC);
            } else {
                flywheel.setVelocity(0);
                flywheelD.setVelocity(0);
            }

            Intake.setPower(gamepad2.left_bumper ? 1.0 : 0.0);

            // =========================== SERVO JUAN ===========================

            // Ajuste manual fino
            boolean juanSubir = gamepad1.right_trigger > 0.1;
            boolean juanBajar = gamepad1.left_trigger > 0.1;
            double velocidadJuan = 0.1;

            if (juanSubir) {
                juanActualPos += velocidadJuan;
            } else if (juanBajar) {
                juanActualPos -= velocidadJuan;
            }

            // Posición fija al pulsar B
            if (gamepad1.y ) {
                juanActualPos = 0.60;
            }

            if (gamepad1.left_bumper ) {
                juanActualPos = 0.3;
            }

            // LÍMITES DE SEGURIDAD PARA JUAN
            if (juanActualPos > JUAN_MAX) juanActualPos = JUAN_MAX;
            if (juanActualPos < JUAN_MIN) juanActualPos = JUAN_MIN;

            Juan.setPosition(juanActualPos);


            // =========================== TELEMETRÍA ===========================

            // RPM del Chasis Principal
            double rpmDI = MotorDI.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmTI = MotorTI.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmDD = MotorDD.getVelocity() * DRIVE_TICKS_TO_RPM;
            double rpmTD = MotorTD.getVelocity() * DRIVE_TICKS_TO_RPM;

            telemetry.addLine("=== DRIVE RPM ===");
            telemetry.addData("DI / TI", "%.0f / %.0f", rpmDI, rpmTI);
            telemetry.addData("DD / TD", "%.0f / %.0f", rpmDD, rpmTD);

            telemetry.addLine("=== LIMELIGHT & APUNTE ===");
            telemetry.addData("Auto-Apunte (Botón X)", autoAimActive);
            telemetry.addData("Tag 20/24 Visible", tagObjetivoVisible);
            telemetry.addData("Tx (Error grados)", "%.2f", tx);

            telemetry.addLine("=== SERVO JUAN ===");
            telemetry.addData("Posición Actual", "%.3f", juanActualPos);

            telemetry.addLine("=== FLYWHEEL PIDF ===");
            telemetry.addData("Activado", flywheelOn);
            telemetry.addData("Target (Ticks/s)", "%.0f", TARGET_TICKS_PER_SEC);
            telemetry.addData("Actual (Izq)", "%.0f", flywheel.getVelocity());

            telemetry.addLine("=== ESTADO SECUENCIA ===");
            telemetry.addData("Disparando", autoFireActivo);
            telemetry.addData("Patrón Mem.", aprilTagId);

            telemetry.update();
        }

        limelight.stop();
    }

    // =========================== FUNCIONES AUXILIARES ===========================

    private PelotaColor detectarPelota(NormalizedColorSensor s1, NormalizedColorSensor s2) {
        float[] hsv = new float[3];
        boolean morado = false, verde = false;

        if (s1 != null) {
            NormalizedRGBA c = s1.getNormalizedColors();
            Color.colorToHSV(c.toColor(), hsv);
            if (esMoradoHSV(hsv[0], hsv[1], hsv[2])) morado = true;
            if (esVerdeHSV (hsv[0], hsv[1], hsv[2])) verde  = true;
        }
        if (s2 != null) {
            NormalizedRGBA c = s2.getNormalizedColors();
            Color.colorToHSV(c.toColor(), hsv);
            if (esMoradoHSV(hsv[0], hsv[1], hsv[2])) morado = true;
            if (esVerdeHSV (hsv[0], hsv[1], hsv[2])) verde  = true;
        }

        if (morado && !verde) return PelotaColor.MORADO;
        if (verde  && !morado) return PelotaColor.VERDE;
        return PelotaColor.NINGUNO;
    }

    private boolean esMoradoHSV(float h, float s, float v) {
        return s >= MIN_SAT && v >= MIN_VAL && h >= MORADO_H_MIN && h <= MORADO_H_MAX;
    }

    private boolean esVerdeHSV(float h, float s, float v) {
        return s >= MIN_SAT && v >= MIN_VAL && h >= VERDE_H_MIN && h <= VERDE_H_MAX;
    }
}
