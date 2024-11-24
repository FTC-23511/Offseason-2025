package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater.startingPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversServo;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

import java.util.List;

public class Robot {
    public SolversMotor liftBottom;
    public SolversMotor liftTop;
    public SolversMotor extension;
    public SolversMotor intakeMotor;

    public SolversMotor leftFront;
    public SolversMotor rightFront;
    public SolversMotor leftBack;
    public SolversMotor rightBack;

    public SolversServo leftIntakePivot;
    public SolversServo rightIntakePivot;

    public SolversServo leftDepositPivot;
    public SolversServo rightDepositPivot;
    public SolversServo depositClaw;

    public Motor.Encoder liftEncoder;
    public Motor.Encoder extensionEncoder;

    public SparkFunOTOSCorrected otos;

    public RevColorSensorV3 colorSensor;

    public List<LynxModule> allHubs;

    public LynxModule ControlHub;

    public Deposit deposit;
    public Intake intake;
    public Drive drive;

    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    // Make sure to run this after instance has been enabled/made
    public void init(HardwareMap hardwareMap) {
        liftBottom = new SolversMotor((hardwareMap.get(DcMotor.class, "liftBottom")), 0.01);
        liftTop = new SolversMotor(hardwareMap.get(DcMotor.class, "liftTop"), 0.01);
        extension = new SolversMotor(hardwareMap.get(DcMotor.class, "extension"), 0.01);
        intakeMotor = new SolversMotor(hardwareMap.get(DcMotor.class, "intakeMotor"), 0.01);

        leftFront = new SolversMotor(hardwareMap.get(DcMotor.class, "FL"), 0.01);
        rightFront = new SolversMotor(hardwareMap.get(DcMotor.class, "FR"), 0.01);
        leftBack = new SolversMotor(hardwareMap.get(DcMotor.class, "BL"), 0.01);
        rightBack = new SolversMotor(hardwareMap.get(DcMotor.class, "BR"), 0.01);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftBottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftEncoder = new Motor(hardwareMap, "liftTop").encoder;
        extensionEncoder = new Motor(hardwareMap, "extension").encoder;

//        liftEncoder.setDirection(Motor.Direction.REVERSE);

        leftIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "leftIntakePivot"), 0.01);
        rightIntakePivot = new SolversServo(hardwareMap.get(Servo.class, "rightIntakePivot"), 0.01);
        leftDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "leftDepositPivot"), 0.01);
        rightDepositPivot = new SolversServo(hardwareMap.get(Servo.class, "rightDepositPivot"), 0.01);
        depositClaw = new SolversServo(hardwareMap.get(Servo.class, "depositClaw"), 0.01);

        leftIntakePivot.setDirection(Servo.Direction.REVERSE);
        leftDepositPivot.setDirection(Servo.Direction.REVERSE);

        colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");

        colorSensor.enableLed(true);

        drive = new Drive(hardwareMap, startingPose);

        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }

        intake = new Intake();
        deposit = new Deposit();
    }

    public void initHasMovement() {
        deposit.init();
        intake.init();
    }
}