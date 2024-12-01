package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.INTAKE_HOLD_SPEED;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.MT2Relocalization;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous
public class MT2RelocalizationTest extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    public GamepadEx driver;
    public GamepadEx operator;
    public static int neededReads = 1;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = Globals.OpModeType.AUTO;

        depositInit = Globals.DepositInit.SPECIMEN_SCORING;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> robot.follower.resetIMU());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().schedule(false, new MT2Relocalization(robot, neededReads));

        telemetry.addData("x", robot.follower.getPose().getX());
        telemetry.addData("y", robot.follower.getPose().getY());
        telemetry.addData("heading", robot.follower.getPose().getHeading());

        telemetry.update();

        super.run();
    }
}