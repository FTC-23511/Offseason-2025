package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

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
import org.firstinspires.ftc.teamcode.subsystem.commands.*;

@Config
@Autonomous
public class FTCLibAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action trajectory;
    Action waitTrajectory;

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.init(hardwareMap);

        robot.initHasMovement();

        trajectory = robot.drive.sparkFunOTOSDrive.actionBuilder(startingPose)
                .strafeToConstantHeading(new Vector2d(38, 58.75))
                .strafeToConstantHeading(new Vector2d(48.5, 58.75))
                .waitSeconds(2.0)
                .build();

        waitTrajectory = robot.drive.sparkFunOTOSDrive.actionBuilder(startingPose)
                .waitSeconds(5.0)
                .build();

        schedule(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new TrajectoryCommand(trajectory, robot)
//                new InstantCommand(() -> robot.deposit.setSlideTarget(HIGH_BUCKET_HEIGHT))
            ),
            new InstantCommand(() -> robot.deposit.setClawOpen(true))
        ));
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