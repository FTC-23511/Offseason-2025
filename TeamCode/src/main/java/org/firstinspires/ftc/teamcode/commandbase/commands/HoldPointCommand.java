package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

public class HoldPointCommand extends CommandBase {
    private final Follower follower;
    private final Pose pose;
    private final boolean fieldCentric;

    /**
     * Moves robot to a new {@link Pose} that is either field or robot centric.
     * @param follower The follower object.
     * @param pose The pose that the robot should go to (see fieldCentric parameter).
     * @param fieldCentric Whether the move should be field centric or robot centric (based off the follower's position at the time of scheduling the command).
     */
    public HoldPointCommand(Follower follower, Pose pose, boolean fieldCentric) {
        this.follower = follower;
        this.pose = pose;
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void initialize() {
        if (!fieldCentric) {
            pose.add(follower.getPose());
        }

        follower.holdPoint(pose);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}