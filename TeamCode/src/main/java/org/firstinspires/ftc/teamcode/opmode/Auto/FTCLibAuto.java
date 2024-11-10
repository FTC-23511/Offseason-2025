package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakePivotState.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.*;

@Config
@Autonomous
public class FTCLibAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action trajectory;

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        startingPose = new Pose2d(8, 61.75, Math.toRadians(270));

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.init(hardwareMap);

        robot.initHasMovement();

        trajectory = robot.drive.actionBuilder(robot.drive.pose)

                .build();
    }

    @Override
    public void run() {
        Actions.runBlocking(
                new ParallelAction(
                        robot.deposit.periodicAction(),
                        robot.intake.periodicAction(),
                        new SequentialAction(
                                trajectory,
                                new FTCLibAction(new setIntake(robot.intake, Intake.IntakePivotState.INTAKE))
                        )
                )
        );
        super.run();
    }
}