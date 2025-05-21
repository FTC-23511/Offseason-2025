package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.DepositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.depositPivotState;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.intakePivotState;
import static org.firstinspires.ftc.teamcode.hardware.Globals.BACK_HIGH_SPECIMEN_ATTACH_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.autoEndPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.depositInit;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Salsa (4spec+0sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class tester extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    public void generatePath(){

        robot.follower.setStartingPose(new Pose(6.125, 66.250, Math.toRadians(0)));


        paths.add(
                robot.follower.pathBuilder()
                        .addPath(
                        new BezierLine(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(26.029, 128.047, Point.CARTESIAN)
                        )
                )
                        .setTangentHeadingInterpolation()
                        .build());

        paths.add(
                robot.follower.pathBuilder()
                    .addPath(
                         new BezierLine(
                                new Point(26.029, 128.047, Point.CARTESIAN),
                                new Point(130.566, 131.405, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(
                robot.follower.pathBuilder()
                    .addPath(
                        new BezierLine(
                                new Point(130.566, 131.405, Point.CARTESIAN),
                                new Point(126.997, 17.843, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(
                robot.follower.pathBuilder()
                    .addPath(
                        new BezierLine(
                                new Point(126.997, 17.843, Point.CARTESIAN),
                                new Point(29.388, 13.854, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());

        paths.add(
                robot.follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(29.388, 13.854, Point.CARTESIAN),
                                new Point(10.076, 84.805, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build());
    }

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositPivotState.FRONT_SPECIMEN_SCORING;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        robot.follower.setMaxPower(0.45);

        generatePath();

        ParallelCommandGroup attachSpecimen = new ParallelCommandGroup(
                new SetDeposit(robot, Deposit.depositPivotState, BACK_HIGH_SPECIMEN_ATTACH_HEIGHT, false),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true))
                )
        );

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.get(0)),
                new FollowPathCommand(robot.follower, paths.get(1)),
                new FollowPathCommand(robot.follower, paths.get(2)),
                new FollowPathCommand(robot.follower, paths.get(3)),
                new FollowPathCommand(robot.follower, paths.get(4))
        );

        dashboardPoseTracker = new DashboardPoseTracker(robot.poseUpdater);
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

    }

    @Override
    public void run() {
        super.run();

        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("extendoReached", robot.intake.extendoReached);
        telemetry.addData("slidesRetracted", robot.deposit.slidesRetracted);
        telemetry.addData("slidesReached", robot.deposit.slidesReached);

        telemetry.addData("intakePivotState", intakePivotState);
        telemetry.addData("depositPivotState", depositPivotState);

        telemetry.addData("UndoTransfer", CommandScheduler.getInstance().isScheduled(new UndoTransfer(robot)));
        telemetry.addData("liftTop.getPower()", robot.liftLeft.getPower());
        telemetry.addData("liftBottom.getPower()", robot.liftRight.getPower());

        telemetry.addData("deposit target", robot.deposit.target);
        telemetry.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());
        telemetry.addData("extendo target", robot.intake.target);
        telemetry.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());

        telemetry.addData("Robot Pose", robot.follower.getPose());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));

        telemetry.addData("Heading Error", Math.toDegrees(robot.follower.headingError));

        telemetry.update(); // DO NOT REMOVE! Needed for telemetry

        // Pathing telemetry
        dashboardPoseTracker.update();
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(robot.poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

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