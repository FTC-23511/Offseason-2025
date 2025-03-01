package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.IntakeMotorState.STOP;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.SampleColorTarget.ALLIANCE_ONLY;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.SampleColorTarget.ANY_COLOR;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;
import org.firstinspires.ftc.teamcode.tuning.example.ExampleRobot;

@TeleOp
public class IntakeTest extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final ExampleRobot robot = ExampleRobot.getInstance();

    private boolean endgame = false;
    private boolean frontSpecimenScoring = false;

    @Override
    public void initialize() {
        // Must have for all opModes
        opModeType = OpModeType.TELEOP;
        depositInit = DepositPivotState.MIDDLE_HOLD;

        INTAKE_HOLD_SPEED = 0;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.exampleIntake);

        robot.exampleIntake.setActiveIntake(STOP);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> robot.exampleIntake.toggleActiveIntake(ANY_COLOR))
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> robot.exampleIntake.toggleActiveIntake(ALLIANCE_ONLY))
        );

        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            INTAKE_HOLD_SPEED = 0.15; // Enable hold

            timer = new ElapsedTime();
            gameTimer = new ElapsedTime();
        }

        if (sampleColor.equals(SampleColorDetected.RED)) {
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("opModeType", opModeType.name());

        telemetryData.addData("hasSample()", robot.exampleIntake.hasSample());
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("Intake sampleColor", sampleColor);
        telemetryData.addData("correctSampleDetected", correctSampleDetected());
        telemetryData.addData("intakeMotorState", intakeMotorState);

        telemetryData.addData("Sigma", "Oscar");

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }
}