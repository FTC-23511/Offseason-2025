package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.List;

public class MT2Relocalization extends CommandBase {
    private final Robot robot;
    private int successfulReads = 0;
    private final int neededReads;
    private final List<Pose> reads = new ArrayList<>();

    public MT2Relocalization(Robot robot, int neededReads) {
        this.robot = robot;
        this.neededReads = neededReads;
    }

    @Override
    public void initialize() {
        robot.limelight.updateRobotOrientation(robot.follower.getPose().getHeading());
        LLResult result = robot.limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT2 = result.getBotpose_MT2();
                successfulReads++;
                reads.add(new Pose(botPoseMT2.getPosition().x, botPoseMT2.getPosition().y, robot.follower.getPose().getHeading()));
            }
        }
    }

    @Override
    public void execute() {
        initialize();
    }

    @Override
    public boolean isFinished() {
        return successfulReads >= neededReads;
    }

    @Override
    public void end(boolean interrupted) {
        try {
            // Averaging
            double x = 0;
            double y = 0;

            for (Pose read : reads) {
                x += read.getX();
                y += read.getY();
            }

            x /= reads.size();
            y /= reads.size();

            // Convert meters to inches
            x *= 39.37008;
            y *= 39.37008;

            // Switch to pedro 0" to 144" coordinate system instead of -72" to 72" coordinate system
            x += 72;
            y += 72;

            robot.follower.setPose(new Pose(x + 72, y, robot.follower.getPose().getHeading()));
        } catch (IndexOutOfBoundsException ignored) {

        }
    }
}
