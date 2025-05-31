package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class AttachSpecimen extends CommandBase {
    Robot robot;
    ElapsedTime timer;

    public AttachSpecimen(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();
        addRequirements(robot.deposit);
    }

    @Override
    public void initialize() {
        robot.deposit.setSlideTarget(HIGH_SPECIMEN_ATTACH_HEIGHT);
        timer.reset();
    }

    @Override
    public void execute() {
        if (robot.deposit.slidesReached) {
            robot.deposit.setClawOpen(true);
        } else {
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return robot.deposit.slidesReached && timer.milliseconds() >= 300;
    }
}
