package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    public ElapsedTime buttonTimer;

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
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when tele-op starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();

            timer = new ElapsedTime();
            buttonTimer = new ElapsedTime();

            // Color Sensor to detect sample in intake
            robot.colorSensor.enableLed(true);
        }
        // Endgame/hang rumble after 105 seconds to notify robot.driver to hang
        else if ((timer.seconds() > 105) && (!endgame)) {
            endgame = true;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

/* Color Sensor Code (Works, but drivers don't want it tho)
        if (Objects.equals(Intake.IntakePivotState.READY_INTAKE, Intake.intakePivotState) || Objects.equals(Intake.IntakePivotState.INTAKE, Intake.intakePivotState)) {
            int red = robot.colorSensor.red();
            int green = robot.colorSensor.green();
            int blue = robot.colorSensor.blue();

            double distance = robot.colorSensor.getDistance(DistanceUnit.CM);
            if (green < 2500) {
                if (distance < 1.5) {
                    if (red >= green && red >= blue) {
                        currentSample = SampleDetected.RED;
                    } else if (green >= red && green >= blue) {
                        currentSample = SampleDetected.YELLOW;
                    } else {
                        currentSample = SampleDetected.BLUE;
                    }
                } else {
                    currentSample = SampleDetected.NONE;
                }
            } else {
                currentSample = SampleDetected.NONE;
            }

            telemetry.addData("RGB", "(" + red + ", " + green + ", " + blue + ")");
        }

        telemetry.update();

        // Gamepad Lights (untested)
        if (currentSample.equals(RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (currentSample.equals(BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (currentSample.equals(YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
*/

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

        // Reset IMU for field centric
        if (gamepad1.x) {
            offset = robot.drive.sparkFunOTOSDrive.pose.heading.toDouble();
        }

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new intakeFullExtendo(robot.intake));

        // Operator Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new realTransfer(robot.deposit, robot.intake));

        operator.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new depositSafeRetracted(robot.deposit));

        if (gamepad2.x && buttonTimer.milliseconds() >= 200) {
            robot.deposit.setClawOpen(!robot.deposit.clawOpen);
            buttonTimer.reset();
        }

        operator.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new attachSpecimen(robot.deposit));

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(Intake.IntakePivotState.INTAKE)));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setPivot(Intake.IntakePivotState.TRANSFER)));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new setDepositScoring(robot.deposit, HIGH_BUCKET_HEIGHT, Deposit.DepositPivotState.SCORING));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new setDepositScoring(robot.deposit, HIGH_SPECIMEN_HEIGHT, Deposit.DepositPivotState.SPECIMEN_SCORING));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new setDepositScoring(robot.deposit, LOW_BUCKET_HEIGHT, Deposit.DepositPivotState.SPECIMEN_SCORING));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new setDepositSlidesIntake(robot.deposit));

        operator.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(() -> robot.intake.setActiveIntake(Intake.IntakeMotorState.FORWARD)));
        operator.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> robot.intake.setActiveIntake(Intake.IntakeMotorState.REVERSE)));

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetry.addData("timer", timer.milliseconds());

        // DO NOT REMOVE! Needed for telemetry
        telemetry.update();
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}