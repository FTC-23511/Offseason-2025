package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class Drive extends SubsystemBase {

    public final Follower follower;

    public Drive(HardwareMap hardwareMap, Pose startingPose) {
        this.follower = new Follower(hardwareMap);;
        this.follower.setStartingPose(startingPose);
    }
}
