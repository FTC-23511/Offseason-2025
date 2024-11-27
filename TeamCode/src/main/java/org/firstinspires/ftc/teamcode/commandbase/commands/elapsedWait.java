package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class elapsedWait extends CommandBase {
    private ElapsedTime timer;
    private double waitTime;

    public elapsedWait(double waitTime) {
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() > waitTime;
    }
}
