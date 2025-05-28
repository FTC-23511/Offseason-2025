package org.firstinspires.ftc.teamcode.tuning.servo;

import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CENTER_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CLAW_SERVO_POS;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.WRIST_SERVO_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class octocanumServosTuner extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static double FL = 0.5;
    public static double FR = 0.5;
    public static double BL = 0.5;
    public static double BR = 0.5;

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        robot.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.FR.setPosition(FR);
        robot.BR.setPosition(BR);
        robot.FL.setPosition(FL);
        robot.BL.setPosition(BL);
    }

    @Override
    public void loop() {
        robot.FR.setPosition(FR);
        robot.BR.setPosition(BR);
        robot.FL.setPosition(FL);
        robot.BL.setPosition(BL);

        telemetry.addData("FR getPosition", robot.FR.getPosition());
        telemetry.addData("BR getPosition", robot.BR.getPosition());
        telemetry.addData("FL getPosition", robot.FL.getPosition());
        telemetry.addData("BL getPosition", robot.BL.getPosition());

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}