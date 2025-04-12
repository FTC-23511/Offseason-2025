package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;


@TeleOp(name = "Limelight Color Detection")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        telemetry.setMsTransmissionInterval(11);

        // pipeline 1 = cuttlefish model
        limelight.pipelineSwitch(1);

        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ta = result.getTa();  // Target area
                double tx = result.getTx();  // Target X position
                double ty = result.getTy();  // Target Y position

                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();

                if (detectorResults != null && !detectorResults.isEmpty()) {
                    double distance = estimateDistance(ta);

                    telemetry.addLine("Primary Target");
                    telemetry.addData("Area", String.format("%.4f", ta));
                    telemetry.addData("Distance", String.format("%.2f in", distance));
                    telemetry.addData("Position (X, Y)", String.format("%.2f, %.2f", tx, ty));

                    telemetry.addLine("\nDetected Samples:");

                    int sampleCount = 0;
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        sampleCount++;

                        String className = dr.getClassName();
                        double area = dr.getTargetArea();
                        double detectedDistance = estimateDistance(area);
                        double xPos = dr.getTargetXDegrees();
                        double yPos = dr.getTargetYDegrees();

                        // Display detection information
                        telemetry.addData(
                                String.format("Sample %d", sampleCount),
                                String.format("Color: %s", className)
                        );

                        telemetry.addData(
                                String.format("  Area %d", sampleCount),
                                String.format("%.4f", area)
                        );

                        telemetry.addData(
                                String.format("  Distance %d", sampleCount),
                                String.format("%.2f in", detectedDistance)
                        );

                        telemetry.addData(
                                String.format("  Position %d", sampleCount),
                                String.format("X: %.2f, Y: %.2f", xPos, yPos)
                        );
                    }
                } else {
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();

                    if (classifierResults != null && !classifierResults.isEmpty()) {
                        telemetry.addLine("\nClassified Samples:");

                        int sampleCount = 0;
                        for (LLResultTypes.ClassifierResult cr : classifierResults) {
                            sampleCount++;

                            String className = cr.getClassName();
                            double confidence = cr.getConfidence();

                            telemetry.addData(
                                    String.format("Sample %d", sampleCount),
                                    String.format("Color: %s, Conf: %.2f", className, confidence)
                            );
                        }
                    } else {
                        telemetry.addLine("No targets detected");
                    }
                }
            } else {
                telemetry.addLine("No valid results from Limelight");
            }

            telemetry.update();

            sleep(50);
        }

        limelight.stop();
    }


    private double estimateDistance(double area) {
        if (area <= 0.0) return -1;

        double referenceArea = 0.25;
        double referenceDistance = 12.0;

        return referenceDistance * (referenceArea / area);
    }
}