package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Objects;

public class MT2Relocalization extends CommandBase {
    private final Robot robot;
    private final int neededReads;
    private final ArrayList<Position> reads = new ArrayList<>();
    ElapsedTime timer;

    public MT2Relocalization(Robot robot, int neededReads) {
        this.robot = robot;
        this.neededReads = neededReads;
    }

    @Override
    public void initialize() {
        if (Objects.equals(timer, null)) {
            timer = new ElapsedTime();
            timer.reset();
        }

        robot.limelight.updateRobotOrientation(robot.follower.getPose().getHeading());
        LLResult result = robot.limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                reads.add(result.getBotpose_MT2().getPosition());
            }
        }
    }

    @Override
    public void execute() {
        initialize();
    }

    @Override
    public boolean isFinished() {
        return (reads.size() >= neededReads) || timer.milliseconds() >= 400;
    }

    @Override
    public void end(boolean interrupted) {
        if (!reads.isEmpty()) {
            // Averaging
            double x = 0;
            double y = 0;

            for (Position read : reads) {
                x += read.x;
                y += read.y;
            }

            x /= reads.size();
            y /= reads.size();

            // Switch to pedro 0" to 144" coordinate system instead of -72" to 72" coordinate system
            x += 72;
            y += 72;

            robot.follower.setPose(new Pose(x , y, robot.follower.getPose().getHeading()));
        }
    }
}
