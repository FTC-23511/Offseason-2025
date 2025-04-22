package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Globals.AllianceColor;

import java.util.List;


@TeleOp(name = "Limelight Color Detection")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;
    private boolean crossWasPressed = false;
    private boolean squareWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // pipeline 1 = cuttlefish model
        limelight.pipelineSwitch(1);

        limelight.start();

        Globals.allianceColor = AllianceColor.RED;

        telemetry.addLine("Alliance Selection");
        telemetry.addLine("Press CROSS for RED Alliance");
        telemetry.addLine("Press SQUARE for BLUE Alliance");
        telemetry.addLine("Current selection: " + Globals.allianceColor);
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross && !crossWasPressed) {
                Globals.allianceColor = AllianceColor.RED;
                telemetry.clear();
                telemetry.addLine("Alliance Selection");
                telemetry.addLine("Press CROSS for RED Alliance");
                telemetry.addLine("Press SQUARE for BLUE Alliance");
                telemetry.addData("Current selection", Globals.allianceColor);
                telemetry.update();
            }
            crossWasPressed = gamepad1.cross;

            if (gamepad1.square && !squareWasPressed) {
                Globals.allianceColor = AllianceColor.BLUE;
                telemetry.clear();
                telemetry.addLine("Alliance Selection");
                telemetry.addLine("Press CROSS for RED Alliance");
                telemetry.addLine("Press SQUARE for BLUE Alliance");
                telemetry.addData("Current selection", Globals.allianceColor);
                telemetry.update();
            }
            squareWasPressed = gamepad1.square;

            sleep(50);
        }

        telemetry.clear();
        telemetry.addData("-", Globals.allianceColor);
        telemetry.update();
        sleep(500);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

                if (detectorResults != null && !detectorResults.isEmpty()) {
                    LLResultTypes.DetectorResult closestSample = findClosestSampleByAlliance(detectorResults);

                    if (closestSample != null) {
                        String className = closestSample.getClassName();
                        double area = closestSample.getTargetArea();
                        double distance = estimateDistance(area);
                        double xPos = closestSample.getTargetXDegrees();
                        double yPos = closestSample.getTargetYDegrees();

                        telemetry.addLine("Closest Alliance Sample");
                        telemetry.addData("Color", className);
                        telemetry.addData("Area", String.format("%.4f", area));
                        telemetry.addData("Distance", String.format("%.2f in", distance));
                        telemetry.addData("Position", String.format("X: %.2f, Y: %.2f", xPos, yPos));
                    } else {
                        telemetry.addLine("No samples matching alliance color found");
                    }

                    telemetry.addData("Alliance", Globals.allianceColor.toString());
                    telemetry.addData("Total Samples Detected", detectorResults.size());
                } else {
                    telemetry.addLine("No targets detected");
                }
            } else {
                telemetry.addLine("No valid results from Limelight");
            }

            telemetry.update();

            sleep(50);
        }

        limelight.stop();
    }


    private LLResultTypes.DetectorResult findClosestSampleByAlliance(List<LLResultTypes.DetectorResult> samples) {
        LLResultTypes.DetectorResult closestSample = null;
        double closestDistance = Double.MAX_VALUE;

        for (LLResultTypes.DetectorResult sample : samples) {
            String color = sample.getClassName().toLowerCase();

            boolean isMatch = false;

            if (Globals.allianceColor == AllianceColor.RED && color.equals("red")) {
                isMatch = true;
            } else if (Globals.allianceColor == AllianceColor.BLUE && color.equals("blue")) {
                isMatch = true;
            }

            if (isMatch) {
                double area = sample.getTargetArea();
                double distance = estimateDistance(area);

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestSample = sample;
                }
            }
        }

        return closestSample;
    }


    private double estimateDistance(double area) {
        if (area <= 0.0) return -1;

        double referenceArea = 0.25;
        double referenceDistance = 12.0;

        return referenceDistance * (referenceArea / area);
    }
}