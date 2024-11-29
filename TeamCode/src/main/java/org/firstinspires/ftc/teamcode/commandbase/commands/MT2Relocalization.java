package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class MT2Relocalization extends CommandBase {
    private final Robot robot;

    public MT2Relocalization(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        robot.limelight.updateRobotOrientation(robot.follower.getPose().getHeading());
        LLResult result = robot.limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botPoseMT2 = result.getBotpose_MT2();
                robot.follower.setPose(new Pose(botPoseMT2.getPosition().x, botPoseMT2.getPosition().y, robot.follower.getPose().getHeading()));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
