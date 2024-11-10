package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public final SparkFunOTOSDrive sparkFunOTOSDrive;

    public Drive(HardwareMap hardwareMap, Pose2d startingPose) {
        sparkFunOTOSDrive = new SparkFunOTOSDrive(hardwareMap, startingPose);
    }


}
