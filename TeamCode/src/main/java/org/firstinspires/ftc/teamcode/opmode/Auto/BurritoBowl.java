package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.commandbase.Deposit.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.commandbase.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commandbase.commands.RealTransfer;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetDeposit;
import org.firstinspires.ftc.teamcode.commandbase.commands.SetIntake;
import org.firstinspires.ftc.teamcode.commandbase.commands.UndoTransfer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;

@Config
@Autonomous(name = "Burrito Bowl (0spec+4sample)", group = "Chipotle Menu", preselectTeleOp = "FullTeleOp")

public class BurritoBowl extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    private final ArrayList<PathChain> paths = new ArrayList<>();

    private DashboardPoseTracker dashboardPoseTracker;
    public void generatePath() {
        // If you want to edit the pathing copy and update the json code/.pp file found in the Recipes package into https://pedro-path-generator.vercel.app/
        // Then paste the following code https://pedro-path-generator.vercel.app/ spits out at you (excluding the top part with the class and constructor headers)
        // Make sure to update the Recipes package so others can update the pathing as well
        // NOTE: .setTangentHeadingInterpolation() doesn't exist its .setTangentHeadingInterpolation() so just fix that whenever you paste

        // Starting Pose (update this as well):
        robot.follower.setStartingPose(new Pose(6.125, 102.125, Math.toRadians(90)));

        paths.add(
                // Drive to first sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 1
                                new BezierCurve(
                                        new Point(6.125, 102.125, Point.CARTESIAN),
                                        new Point(13.726, 104.818, Point.CARTESIAN),
                                        new Point(17.000, 134.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(162)).build());

        paths.add(
                // Drive to second sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 2
                                new BezierLine(
                                        new Point(17.000, 134.000, Point.CARTESIAN),
                                        new Point(22.960, 127.500, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(162)).build());

        paths.add(
                // Drive to second sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 3
                                new BezierLine(
                                        new Point(22.960, 127.500, Point.CARTESIAN),
                                        new Point(17.000, 134.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(162), Math.toRadians(180)).build());

        paths.add(
                // Drive to third sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 4
                                new BezierLine(
                                        new Point(17.000, 134.000, Point.CARTESIAN),
                                        new Point(23.958, 130.250, Point.CARTESIAN)
                                )
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180)).build());

        paths.add(
                // Drive to third sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(23.958, 130.773, Point.CARTESIAN),
                                        new Point(17.000, 134.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160)).build());

        paths.add(
                // Drive to fourth sample intake
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 6
                                new BezierLine(
                                        new Point(17.000, 134.000, Point.CARTESIAN),
                                        new Point(23.709, 132.520, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(-160)).build());

        paths.add(
                // Drive to fourth sample scoring
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(23.709, 132.520, Point.CARTESIAN),
                                        new Point(14.500, 130.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-158), Math.toRadians(135)).build());

        paths.add(
                // Park/ascent level 1
                robot.follower.pathBuilder()
                        .addPath(
                                // Line 8
                                new BezierCurve(
                                        new Point(14.500, 130.000, Point.CARTESIAN),
                                        new Point(62.000, 118.000, Point.CARTESIAN),
                                        new Point(62.000, 95.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(-90)).build());
    }
    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        depositInit = DepositPivotState.SCORING;

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.initHasMovement();

        robot.follower.setMaxPower(0.5);

        generatePath();

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        // Sample 1
                        new ParallelCommandGroup(
                                new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new FollowPathCommand(robot.follower, paths.get(0))
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.FORWARD, 250, true)
                                )
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 2
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(1)),
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 320, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new WaitCommand(250),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new ParallelCommandGroup(
                                new SetIntake(robot, Intake.IntakePivotState.INTAKE, IntakeMotorState.HOLD, 150, true),
                                new FollowPathCommand(robot.follower, paths.get(2))
                        ),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 3
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(3)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 350, true),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new WaitCommand(250),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new FollowPathCommand(robot.follower, paths.get(4)),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        // Sample 4
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(5)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true)
                                )
                        ),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.INTAKE, Intake.IntakeMotorState.FORWARD, 300 , true),
                        new ParallelRaceGroup(
                                new WaitUntilCommand(Intake::correctSampleDetected),
                                new WaitCommand(2000)
                        ),
                        new WaitCommand(250),
                        new RealTransfer(robot),
                        new WaitCommand(250),
                        new SetDeposit(robot, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT, false),
                        new FollowPathCommand(robot.follower, paths.get(6)),
                        new InstantCommand(() -> robot.deposit.setClawOpen(true)),

                        new InstantCommand(() -> robot.follower.setMaxPower(0.7)),

                        // Park
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, paths.get(7)),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SetDeposit(robot, DepositPivotState.AUTO_TOUCH_BAR, 0, false)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.follower.setMaxPower(0.3))
                                )
                        )
                )
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
        telemetry.addData("robotState", Robot.robotState);

        telemetry.addData("intakePivotState", intakePivotState);
        telemetry.addData("depositPivotState", depositPivotState);

        telemetry.addData("UndoTransfer", CommandScheduler.getInstance().isScheduled(new UndoTransfer(robot)));
        telemetry.addData("liftTop.getPower()", robot.liftTop.getPower());
        telemetry.addData("liftBottom.getPower()", robot.liftBottom.getPower());

        telemetry.addData("deposit target", robot.deposit.target);
        telemetry.addData("liftEncoder.getPosition()", robot.liftEncoder.getPosition());
        telemetry.addData("extendo target", robot.intake.target);
        telemetry.addData("extensionEncoder.getPosition()", robot.extensionEncoder.getPosition());

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
}