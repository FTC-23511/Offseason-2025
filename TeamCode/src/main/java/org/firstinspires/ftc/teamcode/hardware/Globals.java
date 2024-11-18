package org.firstinspires.ftc.teamcode.hardware;


import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocation.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import java.util.HashMap;
import java.util.Map;

@Config
public class Globals {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum AllianceColor {
        RED,
        BlUE
    }

    public enum PoseLocation {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static final Map<PoseLocation, Pose2d> STARTING_POSES = new HashMap<PoseLocation, Pose2d>() {{
        put(BLUE_BUCKET, new Pose2d(8, 61.75, Math.toRadians(90)));
        put(BLUE_OBSERVATION, new Pose2d(-8, 61.75, Math.toRadians(90)));
        put(RED_BUCKET, new Pose2d(-8, -61.75, Math.toRadians(270)));
        put(RED_OBSERVATION, new Pose2d(8, -61.75, Math.toRadians(270)));
    }};

    public static OpModeType opModeType;

    public static Pose2d startingPose = new Pose2d(0, 0, 0);
    public static PoseLocation startingPoseName;

    public static AllianceColor allianceColor;

    public static double offset = 0;

    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // By default values refer to servo positions, unless otherwise specified
    // By default for values that control opposite running hardware, the right value of the hardware is used
    // e.g. for ARM_TRANSFER_POS, it should correspond with the real position of the servo at the transfer

    // TODO: TUNE. 9.99 or other sus numbers (like 10,000) generally means not tuned!

    // Intake
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static final double SAMPLE_DISTANCE_THRESHOLD = 2.15;

    public static double INTAKE_PIVOT_TRANSFER_POS = 0.21;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.8;

    // Deposit
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.41;
    public static double DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS = 0.79;
    public static double DEPOSIT_PIVOT_SPECIMEN_SCORING_POS = 0.04;
    public static double DEPOSIT_PIVOT_SCORING_POS = 0.99;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.59;
    public static double DEPOSIT_CLAW_OPEN_POS = 0.625;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.4;
    // Extendo
    public static double MAX_EXTENDO_EXTENSION = 480; // Encoder ticks
    public static double AUTON_EXTENDO_EXTENSION; // Encoder ticks

    // Slides
    public static double MAX_SLIDES_EXTENSION = 1950; // Encoder ticks
    public static double SLIDES_PIVOT_READY_EXTENSION = 200; // Encoder ticks
    public static double LOW_BUCKET_HEIGHT = 1000; // Encoder ticks
    public static double HIGH_BUCKET_HEIGHT = 1950; // Encoder ticks

    public static double SPECIMEN_INTAKE_HEIGHT = 0;
    public static double HIGH_SPECIMEN_HEIGHT = 900; // Encoder ticks
    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = 1050; // Encoder ticks
    public static double AUTO_ASCENT_HEIGHT = 700; // Encoder ticks
    public static double ENDGAME_ASCENT_HEIGHT = 800; // Encoder ticks
}