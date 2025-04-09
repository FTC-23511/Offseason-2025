package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Color Detection")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;
    private int currentPipelineIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        currentPipelineIndex = 0; // 0 = Red detection pipeline
        limelight.pipelineSwitch(currentPipelineIndex);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ta = result.getTa();
                double tx = result.getTx();
                double ty = result.getTy();

                double estimatedDistance = estimateDistance(ta);

                telemetry.addData("Target Area (ta)", ta);
                telemetry.addData("Horizontal Offset (tx)", tx);
                telemetry.addData("Vertical Offset (ty)", ty);
                telemetry.addData("Estimated Distance", estimatedDistance);

                String detectedColor = getDetectedColor(currentPipelineIndex);
                telemetry.addData("Detected Color", detectedColor);
            } else {
                telemetry.addLine("No valid targets detected.");
            }

            telemetry.update();
        }
    }

    private double estimateDistance(double area) {
        if (area <= 0.0) return -1;
        return 1.0 / Math.sqrt(area);
    }

    private String getDetectedColor(int pipelineIndex) {
        switch (pipelineIndex) {
            case 0:
                return "Red";
            case 1:
                return "Blue";
            case 2:
                return "Yellow";
            default:
                return "Unknown";
        }
    }
}
