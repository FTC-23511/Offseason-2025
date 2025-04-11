package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@TeleOp(name = "Limelight Multi-Color Detection")
public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double ta = result.getTa();
                double tx = result.getTx();
                double ty = result.getTy();


                if (ta > 0) {
                    double estimatedDistance = estimateDistance(ta);

                    telemetry.addLine("Detected Target");
                    telemetry.addData("Area", ta);
                    telemetry.addData("tx", tx);
                    telemetry.addData("ty", ty);
                    telemetry.addData("Estimated Distance", estimatedDistance);
                    telemetry.addLine();
                } else {
                    telemetry.addLine("No valid targets detected.");
                }
            } else {
                telemetry.addLine("No valid results.");
            }

            telemetry.update();
        }
    }

    private double estimateDistance(double area) {
        if (area <= 0.0) return -1;
        return 1.0 / Math.sqrt(area);
    }
}
