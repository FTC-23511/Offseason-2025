package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocation.BLUE_BUCKET;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDeposit;

@Config
@Autonomous
public class FTCLibAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action trajectory;
    Action initialStrafe;
    Action waitTrajectory;

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        startingPose = new Pose2d(38, 61.75, 0);

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.init(hardwareMap);

        robot.initHasMovement();

        initialStrafe = robot.drive.sparkFunOTOSDrive.actionBuilder(startingPose)
                .strafeToConstantHeading(new Vector2d(38, 58.75))
                .build();

        trajectory = robot.drive.sparkFunOTOSDrive.actionBuilder(new Pose2d(38, 58.75, 0))
                .strafeToConstantHeading(new Vector2d(38, 58.75))
                .strafeToConstantHeading(new Vector2d(48.5, 58.75))
                .waitSeconds(2.0)

                .strafeToConstantHeading(new Vector2d(47.1, 46.5))
                .turnTo(Math.toRadians(90))
                .waitSeconds(2.0)
                .turnTo(Math.toRadians(45))

                .strafeToConstantHeading(new Vector2d(51.1, 50.5))
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(47.1, 46.5))

                .strafeToLinearHeading(new Vector2d(55, 46.5), Math.toRadians(90))
                .waitSeconds(2.0)
                .strafeToLinearHeading(new Vector2d(47.1, 46.5), Math.toRadians(45))

                .strafeToConstantHeading(new Vector2d(51.1, 50.5))
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(47.1, 46.5))

                .strafeToLinearHeading(new Vector2d(54, 46.5), Math.toRadians(125))
                .waitSeconds(2.0)
                .strafeToLinearHeading(new Vector2d(51.1, 50.5), Math.toRadians(45))

                .strafeToConstantHeading(new Vector2d(51.1, 50.5))
                .waitSeconds(1.5)
                .strafeToConstantHeading(new Vector2d(47.1, 46.5))
                .build();

        waitTrajectory = robot.drive.sparkFunOTOSDrive.actionBuilder(startingPose)
                .waitSeconds(5.0)
                .build();

        CommandScheduler.getInstance().schedule((new SequentialCommandGroup(
            new ParallelCommandGroup(
                new TrajectoryCommand(initialStrafe, robot),
//                new InstantCommand(() -> robot.deposit.setSlideTarget(HIGH_BUCKET_HEIGHT)),
                new setDeposit(robot.deposit, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT)
            )
        )));
    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("target", robot.deposit.target);
        telemetry.addData("depositPos", robot.liftEncoder.getPosition());
        telemetry.addData("liftBottom Motor Power", robot.liftBottom.getPower());
        telemetry.addData("liftTop Motor Power", robot.liftTop.getPower());
        telemetry.update();
    }
}