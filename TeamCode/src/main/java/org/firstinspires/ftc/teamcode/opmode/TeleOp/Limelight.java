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
    private boolean triangleWasPressed = false;
    private boolean circleWasPressed = false;

    private boolean calibrationMode = false;

    private boolean debugMode = true;


    private static final double[][] CALIBRATION_POINTS = {
            {0.1, 6.0},     // 6 inches: area 0.1
            {0.04, 10.0},   // 10 inches: area 0.04
            {0.03, 12.0},   // 12 inches: area 0.03
            {0.01, 24.0}    // 24 inches: area 0.01
    };

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
        telemetry.addLine("Press TRIANGLE to toggle calibration mode");
        telemetry.addLine("Press CIRCLE to toggle debug mode");
        telemetry.addLine("Current selection: " + Globals.allianceColor);
        telemetry.addLine("Calibration mode: " + (calibrationMode ? "ON" : "OFF"));
        telemetry.addLine("Debug mode: " + (debugMode ? "ON" : "OFF"));
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.cross && !crossWasPressed) {
                Globals.allianceColor = AllianceColor.RED;
                updateInitTelemetry();
            }
            crossWasPressed = gamepad1.cross;

            if (gamepad1.square && !squareWasPressed) {
                Globals.allianceColor = AllianceColor.BLUE;
                updateInitTelemetry();
            }
            squareWasPressed = gamepad1.square;

            if (gamepad1.triangle && !triangleWasPressed) {
                calibrationMode = !calibrationMode;
                updateInitTelemetry();
            }
            triangleWasPressed = gamepad1.triangle;

            if (gamepad1.circle && !circleWasPressed) {
                debugMode = !debugMode;
                updateInitTelemetry();
            }
            circleWasPressed = gamepad1.circle;

            sleep(50);
        }

        telemetry.clear();
        telemetry.addData("Calibration mode", calibrationMode ? "ON" : "OFF");
        telemetry.addData("Debug mode", debugMode ? "ON" : "OFF");
        telemetry.update();
        sleep(500);

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

                if (detectorResults != null && !detectorResults.isEmpty()) {
                    if (debugMode) {
                        telemetry.addLine("DEBUG: ALL DETECTIONS");
                        for (int i = 0; i < detectorResults.size(); i++) {
                            LLResultTypes.DetectorResult sample = detectorResults.get(i);
                            String className = sample.getClassName();
                            double area = sample.getTargetArea();
                            double distance = calculateDistanceFromCalibration(area);

                            telemetry.addData("Sample " + i + " Class", className);
                            telemetry.addData("Sample " + i + " Area", String.format("%.6f", area));
                            telemetry.addData("Sample " + i + " Distance", String.format("%.1f in", distance));
                        }
                        telemetry.addLine("END DEBUG");
                    }

                    LLResultTypes.DetectorResult closestSample = findClosestSampleByAlliance(detectorResults);

                    if (closestSample != null) {
                        String className = closestSample.getClassName();
                        double area = closestSample.getTargetArea();
                        double xPos = closestSample.getTargetXDegrees();
                        double yPos = closestSample.getTargetYDegrees();

                        telemetry.addLine("Closest Alliance Sample");
                        telemetry.addData("Color", className);
                        telemetry.addData("Area", String.format("%.6f", area));

                        if (calibrationMode) {
                            telemetry.addLine("CALIBRATION MODE");
                            telemetry.addData("AREA VALUE", String.format("%.6f", area));
                            telemetry.addLine("Place sample at known distances");
                            telemetry.addLine("Record area values for calibration");
                        } else {
                            double distance = calculateDistanceFromCalibration(area);
                            telemetry.addData("Distance", String.format("%.1f in", distance));
                        }

                        telemetry.addData("Position", String.format("X: %.2f, Y: %.2f", xPos, yPos));
                    } else {
                        telemetry.addLine("No samples matching alliance color found");
                        telemetry.addLine("Current alliance: " + Globals.allianceColor);
                        telemetry.addLine("Check detector results in debug section");
                    }

                    telemetry.addData("Alliance", Globals.allianceColor.toString());
                    telemetry.addData("Total Samples Detected", detectorResults.size());
                    telemetry.addData("Calibration mode", calibrationMode ? "ON" : "OFF");
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


    private void updateInitTelemetry() {
        telemetry.clear();
        telemetry.addLine("Alliance Selection Phase");
        telemetry.addLine("Press CROSS for RED Alliance");
        telemetry.addLine("Press SQUARE for BLUE Alliance");
        telemetry.addLine("Press TRIANGLE to toggle calibration mode");
        telemetry.addLine("Press CIRCLE to toggle debug mode");
        telemetry.addData("Current selection", Globals.allianceColor + " ALLIANCE");
        telemetry.addData("Calibration mode", calibrationMode ? "ON" : "OFF");
        telemetry.addData("Debug mode", debugMode ? "ON" : "OFF");
        telemetry.update();
    }


    private LLResultTypes.DetectorResult findClosestSampleByAlliance(List<LLResultTypes.DetectorResult> samples) {
        LLResultTypes.DetectorResult closestSample = null;
        double largestArea = 0.0;

        for (LLResultTypes.DetectorResult sample : samples) {
            String color = sample.getClassName().toLowerCase();

            boolean isMatch = false;

            if (Globals.allianceColor == AllianceColor.RED &&
                    (color.equals("red") || color.equals("RED") || color.equals("Red"))) {
                isMatch = true;
            } else if (Globals.allianceColor == AllianceColor.BLUE &&
                    (color.equals("blue") || color.equals("BLUE") || color.equals("Blue"))) {
                isMatch = true;
            } else if (debugMode && !color.equalsIgnoreCase("red") && !color.equalsIgnoreCase("blue")) {
                telemetry.addData("Unmatched color", color);
            }

            if (isMatch) {
                double area = sample.getTargetArea();

                if (area > largestArea) {
                    largestArea = area;
                    closestSample = sample;
                }
            }
        }

        return closestSample;
    }


    private double calculateDistanceFromCalibration(double area) {
        if (area <= 0.001) return 36.0;
        if (area >= 0.15) return 4.0;

        for (int i = 0; i < CALIBRATION_POINTS.length - 1; i++) {
            double area1 = CALIBRATION_POINTS[i][0];
            double area2 = CALIBRATION_POINTS[i+1][0];

            if (area <= area1 && area >= area2) {
                double dist1 = CALIBRATION_POINTS[i][1];
                double dist2 = CALIBRATION_POINTS[i+1][1];

                double ratio = (area - area2) / (area1 - area2);

                return dist2 - (ratio * (dist2 - dist1));
            }
        }

        if (area < CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][0]) {
            double area1 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 2][0];
            double area2 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][0];
            double dist1 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 2][1];
            double dist2 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][1];

            double slope = (dist2 - dist1) / (area2 - area1);

            return dist2 + (slope * (area2 - area));
        }

        if (area > CALIBRATION_POINTS[0][0]) {
            double area1 = CALIBRATION_POINTS[0][0];
            double area2 = CALIBRATION_POINTS[1][0];
            double dist1 = CALIBRATION_POINTS[0][1];
            double dist2 = CALIBRATION_POINTS[1][1];

            double slope = (dist2 - dist1) / (area2 - area1);

            return dist1 + (slope * (area - area1));
        }

        return 15.0;
    }
}