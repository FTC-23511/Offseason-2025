package org.firstinspires.ftc.teamcode.tuning.example;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.SampleColorDetected.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.SampleColorTarget.*;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleIntake.IntakeMotorState.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class ExampleIntake extends SubsystemBase {
    private final ExampleRobot robot = ExampleRobot.getInstance();
    private final ElapsedTime intakeTimer = new ElapsedTime();
    public boolean pushSampleOut = false;
    public boolean pushSampleIn = false;
    public static double EJECT_TIME = 200;

    private final double divideConstant = 65.0;
    public double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    // Whether the claw is open or not in the current state of the claw
    public enum IntakePivotState {
        INTAKE,
        INTAKE_READY,
        TRANSFER,
        HOVER
    }

    public enum IntakeMotorState {
        REVERSE,
        STOP,
        FORWARD,
        HOLD
    }

    public enum SampleColorTarget {
        ALLIANCE_ONLY,
        ANY_COLOR
    }

    public enum SampleColorDetected {
        RED,
        BLUE,
        YELLOW,
        NONE
    }

    public static SampleColorDetected sampleColor = NONE;
    public static SampleColorTarget sampleColorTarget = ANY_COLOR;
//    public static IntakePivotState intakePivotState = TRANSFER;
    // TODO: make sure you make this transfer again for the real thing
    public static IntakePivotState intakePivotState = INTAKE;
    public static IntakeMotorState intakeMotorState = STOP;

    public void init() {
        robot.colorSensor.enableLed(true);
    }

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakeMotorState.equals(HOLD)) {
            robot.intakeMotor.setPower(INTAKE_HOLD_SPEED);
            ExampleIntake.intakeMotorState = intakeMotorState;
        } else if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    robot.intakeMotor.setPower(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);
                    intakeTimer.reset();
                    break;
            }
            ExampleIntake.intakeMotorState = intakeMotorState;
        }
    }

    public void toggleActiveIntake(SampleColorTarget sampleColorTarget) {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            if (intakeMotorState.equals(FORWARD)) {
                setActiveIntake(STOP);
            } else if (intakeMotorState.equals(STOP) || intakeMotorState.equals(HOLD)) {
                setActiveIntake(FORWARD);
            }
            ExampleIntake.sampleColorTarget = sampleColorTarget;
        }
    }

    public void autoUpdateActiveIntake() {
        if (intakePivotState.equals(INTAKE) || intakePivotState.equals(INTAKE_READY)) {
            switch (intakeMotorState) {
                case FORWARD:
                    if (hasSample()) {
                        sampleColor = sampleColorDetected(robot.colorSensor);
                        if (correctSampleDetected()) {
                            if (pushSampleIn && intakeTimer.milliseconds() > 350) {
                                pushSampleIn = false;
                                setActiveIntake(STOP);
                            } else {
                                intakeTimer.reset();
                                if (!pushSampleOut) {
                                    pushSampleOut = true;
                                }
                                setActiveIntake(REVERSE);
                            }
                        }
                    }
                    break;
                case REVERSE:
                    if (pushSampleOut && intakeTimer.milliseconds() > 350) {
                        setActiveIntake(FORWARD);
                        pushSampleOut = false;
                        pushSampleIn = true;
                        intakeTimer.reset();
                    }
                    if (!hasSample()) {
                        setActiveIntake(STOP);
                    }
                    break;
                // No point of setting intakeMotor to 0 again
            }
        } else if (intakePivotState.equals(TRANSFER)) {
            setActiveIntake(HOLD);
        }
    }

    public static SampleColorDetected sampleColorDetected(RevColorSensorV3 colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (blue >= green && blue >= red) {
            return BLUE;
        } else if (green >= red) {
            return YELLOW;
        } else {
            return RED;
        }
    }

    public static boolean correctSampleDetected() {
        switch (sampleColorTarget) {
            case ANY_COLOR:
                if (sampleColor.equals(YELLOW) ||
                        ((sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE)) ||
                            (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))
                        )
                ) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if ((sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE)) ||
                    (sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))
                ) {
                    return true;
                }
                break;
        }
        return false;
    }

    public boolean hasSample() {
        /* Color thresholding (not used)
        int red = robot.colorSensor.red();
        int green = robot.colorSensor.green();
        int blue = robot.colorSensor.blue();

        SampleColorDetected sampleColor = sampleColorDetected(red, green, blue);

        switch (sampleColor) {
            case YELLOW:
                if (green > YELLOW_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case RED:
                if (red > RED_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
            case BLUE:
                if (blue > BLUE_EDGE_CASE_THRESHOLD) {
                    return false;
                }
                break;
        }
         */

        double distance = robot.colorSensor.getDistance(DistanceUnit.CM);

        return distance > MIN_DISTANCE_THRESHOLD && distance < MAX_DISTANCE_THRESHOLD;
    }

    @Override
    public void periodic() {
        autoUpdateActiveIntake();
    }
}
