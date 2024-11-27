package org.firstinspires.ftc.teamcode.opmode.Auto;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

@Config
@Autonomous(name = "Burrito (1spec+3sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class Burrito extends CommandOpMode {
    private final Robot robot = Robot.getInstance();

    private ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetry;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 78, Math.toRadians(180)));

        paths.add(
                // Drive to first specimen scoring
                robot.follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(6.125, 78.000, Point.CARTESIAN),
                                new Point(40.000, 78.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true).build());

        paths.add(
                // Drive to first sample intake
                robot.follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(38.000, 78.000, Point.CARTESIAN),
                                new Point(22.000, 78.000, Point.CARTESIAN),
                                new Point(42.426, 121.539, Point.CARTESIAN),
                                new Point(22.461, 123.286, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation().build());

        paths.add(
                // Drive to first sample scoring
                robot.follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(22.461, 123.286, Point.CARTESIAN),
                                new Point(15.972, 123.785, Point.CARTESIAN),
                                new Point(12.977, 128.776, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation().build());

        paths.add(
                // Drive to second sample intake
                robot.follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(12.977, 128.776, Point.CARTESIAN),
                                new Point(23.958, 130.773, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(180)).build());

        paths.add(
                // Drive to second sample scoring
                robot.follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(23.958, 130.773, Point.CARTESIAN),
                                new Point(14.974, 130.024, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .setReversed(true).build());

        paths.add(
                // Drive to third sample intake
                robot.follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(14.974, 130.024, Point.CARTESIAN),
                                new Point(23.709, 132.520, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-155)).build());

        paths.add(
                // Drive to third sample scoring
                robot.follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(23.709, 132.520, Point.CARTESIAN),
                                new Point(14.974, 128.776, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-155), Math.toRadians(135)).build());

        paths.add(
                // Park/ascent level 1
                robot.follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(14.974, 128.776, Point.CARTESIAN),
                                new Point(63.706, 117.899, Point.CARTESIAN),
                                new Point(62.157, 94.894, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90)).build());
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

        robot.initHasMovement();

        robot.follower.setMaxPower(0.5);

        generatePath();

        // Put actual auto here
        schedule();

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