package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;

import com.pedropathing.localization.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.UninterruptibleCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commandbase.Drive;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.*;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.TelemetryData;

@TeleOp
public class FullTeleOp extends CommandOpMode {
    public GamepadEx driver;
    public GamepadEx operator;

    public ElapsedTime timer;
    public ElapsedTime gameTimer;

    TelemetryData telemetryData = new TelemetryData(telemetry);

    private final Robot robot = Robot.getInstance();

    private boolean endgame = false;

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
        register(robot.deposit, robot.intake);

        robot.intake.setActiveIntake(IntakeMotorState.STOP);
        Intake.sampleColorTarget = specimenTeleop ? SampleColorTarget.ALLIANCE_ONLY : SampleColorTarget.ANY_COLOR;

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // Driver Gamepad controls
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.STOP, MAX_EXTENDO_EXTENSION, false)
        );

        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> robot.drive.setOctocanumServos(Drive.octocanumServosState == Drive.OctocanumServosState.RETRACTED ? Drive.OctocanumServosState.EXTENDED : Drive.OctocanumServosState.RETRACTED))
        );

        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> robot.follower.setPose(new Pose(0, 0, 0)))
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.REVERSE, robot.intake.target, false)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, ENDGAME_L2_ASCENT_HEIGHT, false)
        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.drive.setOctocanumServos(Drive.OctocanumServosState.PTO_AND_RETRACTED)),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, ENDGAME_L2_ASCENT_HEIGHT, false)
                        ),
                        new InstantCommand(),
                        () -> robot.deposit.target == ENDGAME_L2_ASCENT_HEIGHT
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new ConditionalCommand(
                                new SetDeposit(robot, DepositPivotState.SPECIMEN_SCORING, HIGH_SPECIMEN_HEIGHT, false),
                                new SetDeposit(robot, DepositPivotState.SPECIMEN_INTAKE, INTAKE_SPECIMEN_HEIGHT, true).beforeStarting(new AttachSpecimen(robot)),
                                () -> robot.deposit.target == INTAKE_SPECIMEN_HEIGHT
                        ),
                        new ConditionalCommand(
                                new SetDeposit(robot, DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                                new SetDeposit(robot, DepositPivotState.MIDDLE_HOLD, 0, true),
                                () -> robot.deposit.target == 0
                        ),
                        () -> specimenTeleop
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FORWARD, robot.intake.target, false),
                                new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.FULL_REVERSE, robot.intake.target, true),
                                () -> Intake.intakePivotState.equals(IntakePivotState.TRANSFER)
                        )
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new ConditionalCommand(
                        new SetIntake(robot, IntakePivotState.INTAKE, IntakeMotorState.FULL_REVERSE, MAX_EXTENDO_EXTENSION, true)
                                .andThen(
                                        new SetIntake(robot, IntakePivotState.TRANSFER, IntakeMotorState.HOLD, 0, false)
                                ),
                        new UninterruptibleCommand(
                                new FullTransfer(robot)
                        ),
                        () -> specimenTeleop
                )
        );

        // Swap between sample and specimen teleop
        driver.getGamepadButton(GamepadKeys.Button.PS).whenPressed(
                () -> {
                    specimenTeleop = !specimenTeleop;
                    Intake.sampleColorTarget = specimenTeleop ? SampleColorTarget.ALLIANCE_ONLY : SampleColorTarget.ANY_COLOR;
                }
        );

        // Operator Gamepad controls
        // None right now

        /* Untested TeleOp Automation
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new UninterruptibleCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.follower.setStartingPose(new Pose(6.25, 30, Math.toRadians(180)))),
                                new FollowPathCommand(robot.follower,
                                        robot.follower.pathBuilder()
                                                .addPath(
                                                        new BezierLine(
                                                                new Point(robot.follower.getPose().getX(), robot.follower.getPose().getY(), Point.CARTESIAN),
                                                                new Point(45.757, 69.084, Point.CARTESIAN)
                                                        )
                                                )
                                                .setConstantHeadingInterpolation(Math.toRadians(0)).build(),
                                        true
                                )
                        )
                ).andThen(
                        new InstantCommand(() -> robot.follower.startTeleopDrive())
                )
        );
         */

        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (timer == null) {
            robot.initHasMovement();

            INTAKE_HOLD_SPEED = 0.15; // Enable hold

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
        } else if (sampleColor.equals(SampleColorDetected.BLUE)) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (sampleColor.equals(SampleColorDetected.YELLOW)) {
            gamepad1.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }

        // purple is back (default) spec scoring, green is front spec scoring
        if (specimenTeleop) {
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(1, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }


        if (Drive.octocanumServosState.equals(Drive.OctocanumServosState.PTO_AND_RETRACTED)) {
            robot.follower.setTeleOpMovementVectors(1, 0, 0, true);
        } else {
            // Pinpoint Field Centric Code
            double speedMultiplier = 0.35 + (1 - 0.35) * gamepad1.left_trigger;
            if (Drive.octocanumServosState == Drive.OctocanumServosState.RETRACTED) {
                robot.follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedMultiplier, -gamepad1.left_stick_x * speedMultiplier, -gamepad1.right_stick_x * speedMultiplier, false);
            } else {
                robot.follower.setTeleOpMovementVectors(gamepad1.left_stick_y * speedMultiplier, 0, -gamepad1.right_stick_x * speedMultiplier, true);
            }
            robot.follower.update();
        }

        // Manual control of extendo
        if (gamepad1.right_trigger > 0.01 &&
            !depositPivotState.equals(DepositPivotState.TRANSFER) &&
            robot.intake.getExtendoScaledPosition() <= (MAX_EXTENDO_EXTENSION - 5)) {

            robot.intake.target += 5;
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheduler
        super.run();

        telemetryData.addData("timer", timer.milliseconds());
        telemetryData.addData("autoEndPose", autoEndPose.toString());
        telemetryData.addData("extendoReached", robot.intake.extendoReached);
        telemetryData.addData("extendoRetracted", robot.intake.extendoRetracted);
        telemetryData.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetryData.addData("slidesReached", robot.deposit.slidesReached);
        telemetryData.addData("opModeType", opModeType.name());

        telemetryData.addData("hasSample()", robot.intake.hasSample());
        telemetryData.addData("transferring", robot.intake.transferring);
        telemetryData.addData("Intake Motor Current Draw", robot.intakeMotor.getCurrent(CurrentUnit.AMPS));
        telemetryData.addData("colorSensor getDistance", robot.colorSensor.getDistance(DistanceUnit.CM));
        telemetryData.addData("Intake sampleColor", Intake.sampleColor);
        telemetryData.addData("correctSampleDetected", Intake.correctSampleDetected());
        telemetryData.addData("intakeMotorState", Intake.intakeMotorState);

        telemetryData.addData("liftTop.getPower()", robot.liftLeft.getPower());
        telemetryData.addData("liftBottom.getPower()", robot.liftRight.getPower());
        telemetryData.addData("extension.getPower()", robot.extension.getPower());

        telemetryData.addData("getExtendoScaledPosition()", robot.intake.getExtendoScaledPosition());
        telemetryData.addData("getLiftScaledPosition()", robot.deposit.getLiftScaledPosition());

        telemetryData.addData("slides target", robot.deposit.target);
        telemetryData.addData("extendo target", robot.intake.target);

        telemetryData.addData("intakePivotState", intakePivotState);
        telemetryData.addData("depositPivotState", depositPivotState);
        telemetryData.addData("Sigma", "Veer Nahar");

        telemetryData.update(); // DO NOT REMOVE! Needed for telemetry
        timer.reset();
        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        robot.ControlHub.clearBulkCache();
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}