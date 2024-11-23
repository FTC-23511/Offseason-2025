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
        BLUE
    }

    public enum PoseLocation {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static final Map<PoseLocation, Pose2d> STARTING_POSES = new HashMap<PoseLocation, Pose2d>() {{
        put(BLUE_BUCKET, new Pose2d(38, 61.75, Math.toRadians(0)));
        put(BLUE_OBSERVATION, new Pose2d(-8, 61.75, Math.toRadians(90)));
        put(RED_BUCKET, new Pose2d(-8, -61.75, Math.toRadians(270)));
        put(RED_OBSERVATION, new Pose2d(8, -61.75, Math.toRadians(270)));
    }};

    public static OpModeType opModeType;

    public static Pose2d startingPose = new Pose2d(0, 0, 0);
    public static PoseLocation startingPoseName = null;
    public static AllianceColor allianceColor;

    public static double offset = 0;

    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 11.5;
    public static double ROBOT_LENGTH = 12.25;

    // Intake Motor
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -1.0;
    public static double INTAKE_HOLD_SPEED = 0.15;
    public static final double SAMPLE_DISTANCE_THRESHOLD = 2.15;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.21;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.8;

    // Intake Extendo
    public static double MAX_EXTENDO_EXTENSION = 480;
    public static double AUTO_EXTENDO_EXTENSION;

    // Deposit Pivot
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.395;
    public static double DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS = 0.78;
    public static double DEPOSIT_PIVOT_SPECIMEN_SCORING_POS = 0.00;
    public static double DEPOSIT_PIVOT_SCORING_POS = 1.0;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.58;

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.625;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.4;

    // Deposit Slides
    public static double MAX_SLIDES_EXTENSION = 2050;
    public static double SLIDES_PIVOT_READY_EXTENSION = 200;
    public static double LOW_BUCKET_HEIGHT = 1000;
    public static double HIGH_BUCKET_HEIGHT = 2050;
    public static double HIGH_SPECIMEN_HEIGHT = 900;
    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = 1350;
    public static double AUTO_ASCENT_HEIGHT = 700;
    public static double ENDGAME_ASCENT_HEIGHT = 1150;
}