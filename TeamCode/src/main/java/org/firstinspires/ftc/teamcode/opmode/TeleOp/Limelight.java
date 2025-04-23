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

    // Calibration mode - set to true to show raw area values for calibration
    private boolean calibrationMode = false;

    // Calibration data from your exact measurements
    // Format: {area, distance}
    private static final double[][] CALIBRATION_POINTS = {
            {0.13, 5.0},    // 5 inches: area 0.13 (max value)
            {0.088, 7.0},   // 7 inches: area 0.088 (max value)
            {0.04, 10.0},   // 10 inches: area 0.04 (max value)
            {0.032, 12.0},  // 12 inches: area 0.032 (max value)
            {0.01, 24.0}    // 24 inches: area 0.01 (max value)
    };

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // pipeline 1 = cuttlefish model
        limelight.pipelineSwitch(1);

        limelight.start();

        // Set default alliance color
        Globals.allianceColor = AllianceColor.RED;

        // Initialize alliance selection phase
        telemetry.addLine("=== Alliance Selection Phase ===");
        telemetry.addLine("Press CROSS (X) for RED Alliance");
        telemetry.addLine("Press SQUARE (□) for BLUE Alliance");
        telemetry.addLine("Press TRIANGLE (△) to toggle calibration mode");
        telemetry.addLine("Current selection: " + Globals.allianceColor);
        telemetry.addLine("Calibration mode: " + (calibrationMode ? "ON" : "OFF"));
        telemetry.addLine("You can switch until you press START");
        telemetry.update();

        // Wait for the start button to be pressed
        while (!isStarted() && !isStopRequested()) {
            // Check for RED alliance selection (CROSS button)
            if (gamepad1.cross && !crossWasPressed) {
                Globals.allianceColor = AllianceColor.RED;
                updateInitTelemetry();
            }
            crossWasPressed = gamepad1.cross;

            // Check for BLUE alliance selection (SQUARE button)
            if (gamepad1.square && !squareWasPressed) {
                Globals.allianceColor = AllianceColor.BLUE;
                updateInitTelemetry();
            }
            squareWasPressed = gamepad1.square;

            // Toggle calibration mode (TRIANGLE button)
            if (gamepad1.triangle && !triangleWasPressed) {
                calibrationMode = !calibrationMode;
                updateInitTelemetry();
            }
            triangleWasPressed = gamepad1.triangle;

            // Small sleep to prevent CPU overuse
            sleep(50);
        }

        // At this point START has been pressed, show final selection
        telemetry.clear();
        telemetry.addData("MATCH STARTED with", Globals.allianceColor + " ALLIANCE");
        telemetry.addData("Calibration mode", calibrationMode ? "ON" : "OFF");
        telemetry.update();
        sleep(500); // Brief pause to see the selection

        // Main loop
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

                if (detectorResults != null && !detectorResults.isEmpty()) {
                    // Find the closest sample matching alliance color
                    LLResultTypes.DetectorResult closestSample = findClosestSampleByAlliance(detectorResults);

                    if (closestSample != null) {
                        // Get information about the closest matching sample
                        String className = closestSample.getClassName();
                        double area = closestSample.getTargetArea();
                        double xPos = closestSample.getTargetXDegrees();
                        double yPos = closestSample.getTargetYDegrees();

                        // Display detection information for closest sample
                        telemetry.addLine("Closest Alliance Sample");
                        telemetry.addData("Color", className);
                        telemetry.addData("Area", String.format("%.6f", area));

                        if (calibrationMode) {
                            // In calibration mode, show raw area value prominently
                            telemetry.addLine("=== CALIBRATION MODE ===");
                            telemetry.addData("AREA VALUE", String.format("%.6f", area));
                            telemetry.addLine("Place sample at known distances");
                            telemetry.addLine("Record area values for calibration");
                        } else {
                            // Calculate distance using precise calibration data
                            double distance = calculateDistanceFromCalibration(area);
                            telemetry.addData("Distance", String.format("%.1f in", distance));
                        }

                        telemetry.addData("Position", String.format("X: %.2f, Y: %.2f", xPos, yPos));
                    } else {
                        telemetry.addLine("No samples matching alliance color found");
                    }

                    // Display alliance color and total samples detected (for debugging)
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

    /**
     * Update the telemetry during initialization
     */
    private void updateInitTelemetry() {
        telemetry.clear();
        telemetry.addLine("=== Alliance Selection Phase ===");
        telemetry.addLine("Press CROSS (X) for RED Alliance");
        telemetry.addLine("Press SQUARE (□) for BLUE Alliance");
        telemetry.addLine("Press TRIANGLE (△) to toggle calibration mode");
        telemetry.addData("Current selection", Globals.allianceColor + " ALLIANCE");
        telemetry.addData("Calibration mode", calibrationMode ? "ON" : "OFF");
        telemetry.addLine("You can switch until you press START");
        telemetry.update();
    }

    /**
     * Find the closest sample matching the alliance color
     * @param samples List of detected samples
     * @return The closest sample matching alliance color, or null if none found
     */
    private LLResultTypes.DetectorResult findClosestSampleByAlliance(List<LLResultTypes.DetectorResult> samples) {
        LLResultTypes.DetectorResult closestSample = null;
        double largestArea = 0.0; // Use area directly - larger area = closer

        for (LLResultTypes.DetectorResult sample : samples) {
            String color = sample.getClassName().toLowerCase();

            // Check if sample color matches alliance color
            boolean isMatch = false;

            if (Globals.allianceColor == AllianceColor.RED && color.equals("red")) {
                isMatch = true;
            } else if (Globals.allianceColor == AllianceColor.BLUE && color.equals("blue")) {
                isMatch = true;
            }

            // If it's a match, check if it has a larger area (is closer) than current
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

    /**
     * Calculate distance using precise calibration data collected from real measurements
     * @param area Detected area from the Limelight
     * @return Distance in inches
     */
    private double calculateDistanceFromCalibration(double area) {
        // Handle edge cases
        if (area <= 0.001) return 36.0; // Very far away (beyond calibration)
        if (area >= 0.2) return 4.0;    // Very close (beyond calibration)

        // Find which calibration segment the area falls in
        for (int i = 0; i < CALIBRATION_POINTS.length - 1; i++) {
            double area1 = CALIBRATION_POINTS[i][0];
            double area2 = CALIBRATION_POINTS[i+1][0];

            if (area <= area1 && area >= area2) {
                // Found the segment, now interpolate
                double dist1 = CALIBRATION_POINTS[i][1];
                double dist2 = CALIBRATION_POINTS[i+1][1];

                // Calculate how far between the two points (0.0 to 1.0)
                double ratio = (area - area2) / (area1 - area2);

                // Interpolate the distance
                return dist2 - (ratio * (dist2 - dist1));
            }
        }

        // If area is smaller than our last calibration point (farther away)
        if (area < CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][0]) {
            // Extrapolate using the last two points
            double area1 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 2][0];
            double area2 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][0];
            double dist1 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 2][1];
            double dist2 = CALIBRATION_POINTS[CALIBRATION_POINTS.length - 1][1];

            // Calculate the slope of the line
            double slope = (dist2 - dist1) / (area2 - area1);

            // Extrapolate distance
            return dist2 + (slope * (area2 - area));
        }

        // If area is larger than our first calibration point (closer)
        if (area > CALIBRATION_POINTS[0][0]) {
            // Extrapolate using the first two points
            double area1 = CALIBRATION_POINTS[0][0];
            double area2 = CALIBRATION_POINTS[1][0];
            double dist1 = CALIBRATION_POINTS[0][1];
            double dist2 = CALIBRATION_POINTS[1][1];

            // Calculate the slope of the line
            double slope = (dist2 - dist1) / (area2 - area1);

            // Extrapolate distance
            return dist1 + (slope * (area - area1));
        }

        // Should never reach here, but return a reasonable value
        return 15.0;
    }
}