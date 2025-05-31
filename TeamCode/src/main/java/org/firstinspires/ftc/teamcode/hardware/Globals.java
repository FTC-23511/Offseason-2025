package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;

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

    public enum PoseLocationName {
        BLUE_BUCKET,
        BLUE_OBSERVATION,
        RED_BUCKET,
        RED_OBSERVATION
    }

    public static Deposit.DepositPivotState depositInit;

    public static OpModeType opModeType;
    public static boolean specimenTeleop = false;
    public static AllianceColor allianceColor;
    public static PoseLocationName poseLocationName;

    public static Pose subSample1 = new Pose(62.000, 93.700, Math.toRadians(90));
    public static Pose subSample2 = new Pose(62.000, 93.700, Math.toRadians(90));
    public static Pose autoEndPose = new Pose(0, 0, Math.toRadians(0));


    // Robot Width and Length (in inches)
    public static double ROBOT_WIDTH = 13.5;
    public static double ROBOT_LENGTH = 12.15;

    // Intake Motor
    public static double INTAKE_FORWARD_SPEED = 1.0;
    public static double INTAKE_REVERSE_SPEED = -0.5;
    public static double INTAKE_HOLD_SPEED = 0;
    public static int REVERSE_TIME_MS = 300;

    // Intake Color Sensor
    public static double MIN_DISTANCE_THRESHOLD = 1.0;
    public static double MAX_DISTANCE_THRESHOLD = 1.5;
    public static int YELLOW_THRESHOLD = 800;
    public static int RED_THRESHOLD = 0;
    public static int BLUE_THRESHOLD = 0;

    public static int YELLOW_EDGE_CASE_THRESHOLD = 1450;
    public static int RED_EDGE_CASE_THRESHOLD = 700;
    public static int BLUE_EDGE_CASE_THRESHOLD = 675;

    // Intake Pivot
    public static double INTAKE_PIVOT_TRANSFER_POS = 0.15;
    public static double INTAKE_PIVOT_INTAKE_POS = 0.30;
    public static double INTAKE_PIVOT_HOVER_INTAKE_POS = 0.30;

    // Intake Extendo
    public static double MAX_EXTENDO_EXTENSION = 500;

    // Deposit Pivot
    public static double DEPOSIT_PIVOT_TRANSFER_POS = 0.6;
    public static double DEPOSIT_PIVOT_READY_TRANSFER_POS = 0.90;
    public static double DEPOSIT_PIVOT_MIDDLE_POS = 0.55;
    public static double DEPOSIT_PIVOT_AUTO_BAR_POS = 0.35;
    public static double DEPOSIT_PIVOT_SCORING_POS = 0.12;
    public static double DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS = 1.00;
    public static double DEPOSIT_PIVOT_SPECIMEN_SCORING_POS = 0.55;

    // 0.84 sec/360° -> 0.828 sec/355° -> 828 milliseconds/355°
    public static double DEPOSIT_PIVOT_MOVEMENT_TIME = 828 + 200; // 200 milliseconds of buffer
    // 0.84 sec/360° -> 0.828 sec/355° -> (gear ratio of 48:80) 0.497 sec/355° -> 497 milliseconds/355°
    public static double INTAKE_PIVOT_MOVEMENT_TIME = 497 + 200; // 200 milliseconds of buffer

    // Deposit Claw
    public static double DEPOSIT_CLAW_OPEN_POS = 0.74;
    public static double DEPOSIT_CLAW_CLOSE_POS = 0.47;

    // Deposit Wrist
    public static double WRIST_SCORING = 0.7;
    public static double WRIST_AUTO_BAR = 0.3;
    public static double WRIST_SPECIMEN_SCORING = 0.42;
    public static double WRIST_SPECIMEN_INTAKE = 0.32;
    public static double WRIST_TRANSFER = 0.33;
    public static double WRIST_MIDDLE_HOLD = 0.3;

    // Deposit Slides
    public static double MAX_SLIDES_EXTENSION = 1650;
    public static double SLIDES_PIVOT_READY_EXTENSION = 450;
    public static double LOW_BUCKET_HEIGHT = 450;
    public static double HIGH_BUCKET_HEIGHT = 1550;
    public static double HIGH_SPECIMEN_HEIGHT = 850;

    public static double HIGH_SPECIMEN_ATTACH_HEIGHT = HIGH_SPECIMEN_HEIGHT + 200;
    public static double INTAKE_SPECIMEN_HEIGHT = 400;
    public static double AUTO_ASCENT_HEIGHT = 800;
    public static double ENDGAME_L2_ASCENT_HEIGHT = 900;
    public static double ENDGAME_L3_ASCENT_HEIGHT = 1300;

    // Octocanum Servos
    public static double FR_RETRACTED = 0;
    public static double FR_EXTENDED = 0.5;
    public static double FR_PTO_RETRACTED = 1;
    public static double FL_RETRACTED = 1;
    public static double FL_EXTENDED = 0.5;
    public static double FL_PTO_RETRACTED = 0;
    public static double BR_RETRACTED = 0.5;
    public static double BR_EXTENDED = 0;
    public static double BR_PTO_RETRACTED = BR_RETRACTED;
    public static double BL_RETRACTED = 0;
    public static double BL_EXTENDED = 0.5;
    public static double BL_PTO_RETRACTED = BL_RETRACTED;


    // command timeout
    public final static int MAX_COMMAND_RUN_TIME_MS = 3000;
}