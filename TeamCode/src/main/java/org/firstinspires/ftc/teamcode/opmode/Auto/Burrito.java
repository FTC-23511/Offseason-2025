package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDeposit;

@Config
@Autonomous(name = "Burrito (1spec+3sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class Burrito extends CommandOpMode {
    private final Robot robot = Robot.getInstance();

    private PathChain path;

    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetry;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentialHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 78, 0));

        path = robot.follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.125, 78.000, Point.CARTESIAN),
                                new Point(40.000, 78.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation().build();
//                .addPath(
//                        // Line 2
//                        new BezierCurve(
//                                new Point(40.000, 78.000, Point.CARTESIAN),
//                                new Point(22.000, 78.000, Point.CARTESIAN),
//                                new Point(42.426, 121.539, Point.CARTESIAN),
//                                new Point(22.461, 123.286, Point.CARTESIAN)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
//                .addPath(
//                        // Line 3
//                        new BezierCurve(
//                                new Point(22.461, 123.286, Point.CARTESIAN),
//                                new Point(15.972, 123.785, Point.CARTESIAN),
//                                new Point(12.977, 128.776, Point.CARTESIAN)
//                        )
//                )
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
//                .addPath(
//                        // Line 4
//                        new BezierLine(
//                                new Point(12.977, 128.776, Point.CARTESIAN),
//                                new Point(23.958, 130.773, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-57), Math.toRadians(0))
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(23.958, 130.773, Point.CARTESIAN),
//                                new Point(14.974, 130.024, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//                .setReversed(true)
//                .addPath(
//                        // Line 6
//                        new BezierLine(
//                                new Point(14.974, 130.024, Point.CARTESIAN),
//                                new Point(23.709, 132.520, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(25))
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(23.709, 132.520, Point.CARTESIAN),
//                                new Point(14.974, 128.776, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-45))
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(14.974, 128.776, Point.CARTESIAN),
//                                new Point(63.706, 117.899, Point.CARTESIAN),
//                                new Point(62.157, 94.894, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90)).build();
    }
    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositInit.SPECIMEN_SCORING;

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.init(hardwareMap);

        robot.initHasMovement();

        generatePath();

        schedule(
                new RunCommand(() -> robot.follower.update()),
                new SequentialCommandGroup(
                        new FollowPathCommand(robot.follower, path)
//                        new setDeposit(robot.deposit, Deposit.DepositPivotState.SCORING, LOW_BUCKET_HEIGHT)
                )
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.follower.poseUpdater);
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    @Override
    public void run() {
        super.run();

        // Pathing telemetry
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.follower.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}