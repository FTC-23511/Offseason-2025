package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.hardware.Globals.startingPoseName;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class    Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    // Whether the claw is open or not in the current state of the claw
    public boolean clawOpen = true;

    public enum ClawState {
        INNER,
        OUTER
    }

    public ClawState clawState;

    public enum IntakePivotState {
        READY_INTAKE,
        INTAKE,
        TRANSFER,
        MIDDLE_HOLD
    }

    public enum IntakeMotorState {
        REVERSE,
        STOP,
        FORWARD
    }

    public static String sampleColor = "NONE";
    public static IntakePivotState intakePivotState;
    public static IntakeMotorState intakeMotorState;
    private static final PIDFController extendoPIDF = new PIDFController(0.023,0,0, 0.001);

    public void init() {
        setPivot(IntakePivotState.MIDDLE_HOLD);
        setExtendoTarget(0);
        extendoPIDF.setTolerance(15);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(robot.extensionEncoder.getPosition(), this.target);

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0) {
            extendoPower -= 0.1;
        }

        extendoReached = extendoPIDF.atSetPoint();
        extendoRetracted = (target <= 0) && extendoReached;

        if (extendoReached) {
            robot.extension.setPower(0);
        } else {
            robot.extension.setPower(extendoPower);
        }
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public void stopSlide() {
        robot.extension.setPower(0);
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch (intakePivotState) {
            case READY_INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_READY_PICKUP_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_READY_PICKUP_POS);
                break;
            case TRANSFER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                break;
            case MIDDLE_HOLD:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_HOLD_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_HOLD_POS);
                break;
            case INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_PICKUP_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_PICKUP_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        switch (intakeMotorState) {
            case FORWARD:
                robot.intakeMotor.setPower(1);
            case REVERSE:
                robot.intakeMotor.setPower(-1);
            case STOP:
                robot.intakeMotor.setPower(0);
        }

        Intake.intakeMotorState = intakeMotorState;
    }

    public void autoUpdateActiveIntake(IntakeMotorState intakeMotorState) {
        switch (intakeMotorState) {
            case FORWARD:
                if (robot.colorSensor.getDistance(DistanceUnit.CM) < SAMPLE_DISTANCE_THRESHOLD) {
                    sampleColor = sampleDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                    if (correctSampleDetected(sampleColor, startingPoseName)) {
                        robot.intakeMotor.setPower(0);
                    } else if (!sampleColor.equals("NONE")) {
                        robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                        Intake.intakeMotorState = IntakeMotorState.REVERSE;
                    }
                }
            case REVERSE:
                robot.intakeMotor.setPower(-1);
            case STOP:
                robot.intakeMotor.setPower(0);
        }
    }

    public static String sampleDetected(int red, int green, int blue) {
        if ((blue + green + red) >= 900) {
            if (blue >= green && blue >= red) {
                return "BLUE";
            } else if (green >= red) {
                return "YELLOW";
            } else {
                return "RED";
            }
        }
        else {
            return "NONE";
        }
    }

    public static boolean correctSampleDetected(String sampleColor, PoseLocation startingPoseName) {
        return (((sampleColor.equals("BLUE") && (startingPoseName.equals(PoseLocation.BLUE_BUCKET) || startingPoseName.equals(PoseLocation.BLUE_OBSERVATION))) || (sampleColor.equals("RED") && (startingPoseName.equals(PoseLocation.RED_BUCKET) || startingPoseName.equals(PoseLocation.RED_OBSERVATION)))) || sampleColor.equals("YELLOW"));
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
        autoUpdateActiveIntake(intakeMotorState);
    }
}
