package org.firstinspires.ftc.teamcode.opmode.Auto;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.*;

@Config
@Autonomous
public class FTCLibAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    Action trajectory;

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        startingPose = new Pose2d(-8, 61.75, Math.toRadians(90));

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        register(robot.deposit, robot.intake);

        robot.init(hardwareMap);

        robot.initHasMovement();

        trajectory = robot.drive.sparkFunOTOSDrive.actionBuilder(robot.drive.sparkFunOTOSDrive.pose)
                .strafeToConstantHeading(new Vector2d(-8, 32.75))
                .waitSeconds(2.0)
                .strafeToConstantHeading(new Vector2d(-8, 39.75))
                .splineToConstantHeading(new Vector2d(-34.5, 10.1), Math.toRadians(273))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-46, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-53, 50.1))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(-53, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 10.1))
                .strafeToConstantHeading(new Vector2d(-61, 54.5))
                .build();
                            
        schedule(new TrajectoryCommand(trajectory, robot));
    }
}