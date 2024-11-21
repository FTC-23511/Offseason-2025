package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.*;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;
    public SparkFunOTOSDrive drive;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        startingPose = new Pose2d(0, 0, 0);

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(Intake.SampleColorTarget.ANY_COLOR)));

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.intake.toggleActiveIntake(SampleColorTarget.ALLIANCE_ONLY)));

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> offset = robot.drive.sparkFunOTOSDrive.pose.heading.toDouble()));

        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new setExtendo(robot.deposit, robot.intake, MAX_EXTENDO_EXTENSION));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SPECIMEN_SCORING, ENDGAME_ASCENT_HEIGHT));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SPECIMEN_SCORING, 0));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(Intake.IntakePivotState.TRANSFER)));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(Intake.IntakePivotState.INTAKE)));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new extendoSampleEject(robot.deposit, robot.intake));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new realTransfer(robot.deposit, robot.intake));

        // Operator Gamepad controls
        operator.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> robot.deposit.setClawOpen(!robot.deposit.clawOpen)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SCORING, LOW_BUCKET_HEIGHT));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SPECIMEN_SCORING, HIGH_SPECIMEN_HEIGHT));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.INTAKE, 0));

        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new attachSpecimen(robot.deposit));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new setDeposit(robot.deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0));

        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();

            timer = new ElapsedTime();
            gameTimer = new ElapsedTime();
        }
        // Endgame/hang rumble after 105 seconds to notify robot.driver to hang
        else if ((gameTimer.seconds() > 105) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

        if (sampleColor.equals(SampleColorDetected.RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // OTOS Field Centric robot.Drive Code
        robot.drive.sparkFunOTOSDrive.updatePoseEstimate();
        robot.drive.sparkFunOTOSDrive.setFieldCentricDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                        (gamepad1.left_stick_y),
                        (gamepad1.left_stick_x)
                    ),
                gamepad1.right_stick_x
            ),
            gamepad1.left_trigger,
            (robot.drive.sparkFunOTOSDrive.pose.heading.toDouble() - offset)
        );

        if (gamepad1.right_trigger > 0.01 &&
            !Deposit.depositPivotState.equals(Deposit.DepositPivotState.TRANSFER) &&
            robot.extensionEncoder.getPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {

            robot.intake.target += 5;
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("offset", offset);
        telemetry.addData("offset (double)", robot.drive.sparkFunOTOSDrive.pose.heading.toDouble());
        telemetry.addData("offset (radian)", robot.drive.sparkFunOTOSDrive.pose.heading);


        telemetry.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}