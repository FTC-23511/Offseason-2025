package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.caching.SolversMotor;

@TeleOp
@Config
public class ActiveIntake extends LinearOpMode {
    public static double outSpeed = -0.32;

    @Override
    public void runOpMode() throws InterruptedException {
        SolversMotor intakeMotor = new SolversMotor((hardwareMap.get(DcMotor.class, "intakeMotor")), 0.01);
        RevColorSensorV3 colorSensor = (RevColorSensorV3) hardwareMap.colorSensor.get("colorSensor");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor.enableLed(true);

        ElapsedTime timer = new ElapsedTime();

        int intakeMoving = 0;  // -1: reverse, 0: stopped, 1: forward

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        String color;
        double distance = colorSensor.getDistance(DistanceUnit.CM);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            timer.reset();

            if (gamepad1.cross && intakeMoving != 1) {
                intakeMotor.setPower(1);
                intakeMoving = 1;
            } else if (gamepad1.square && intakeMoving != -1) {
                intakeMotor.setPower(outSpeed);
                intakeMoving = -1;
            }

            if (intakeMoving == 1) {
                distance = colorSensor.getDistance(DistanceUnit.CM);
                if (distance < 2.15) {
                    red = colorSensor.red(); // replace with blue for red side teleop

                    green = colorSensor.green();
                    blue = colorSensor.blue();

                    color = sampleDetected(red, green, blue);
                    telemetry.addData("color", color);

                    if (color.equals("RED")) {  // replace with blue for red side teleop
                        intakeMotor.setPower(outSpeed);
                        intakeMoving = -1;
                    } else if (color.equals("BLUE") || color.equals("YELLOW")) {  // replace with red & yellow for red side teleop
                        intakeMotor.setPower(0);
                        intakeMoving = 0;
                    }
                }
            } else if (intakeMoving == -1) {
                distance = colorSensor.getDistance(DistanceUnit.CM);
                if (distance > 2.15) {
                    intakeMotor.setPower(0);
                    intakeMoving = 0;
            }
        }

        telemetry.addData("RGB", "(" + red + ", " + green + ", " + blue + ")");
        telemetry.addData("distance (CM)", distance);
        telemetry.addData("intake moving", intakeMoving);
        telemetry.addData("loop time (ms)", timer.milliseconds());
        telemetry.update();
        }
    }


    public static String sampleDetected(int red, int green, int blue) {
        if ((blue + green + red) >= 900) {
            if (blue >= green && blue >= red) {
                return "BLUE";
            } else if (green >= red) {
                return "YELLOW";
            } else {
                return "RED";
            }
        }
        else {
            return "NONE";
        }
    }
}