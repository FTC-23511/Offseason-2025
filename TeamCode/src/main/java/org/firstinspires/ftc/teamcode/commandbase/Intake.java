package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorDetected.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.SampleColorTarget.*;
import static org.firstinspires.ftc.teamcode.commandbase.Intake.IntakeMotorState.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    public double target;
    public boolean extendoReached;
    // Between retracted and extended
    public boolean extendoRetracted;
    // Between transfer and intake position
    // Whether the claw is open or not in the current state of the claw
    public enum IntakePivotState {
        INTAKE,
        TRANSFER
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
    public static IntakePivotState intakePivotState = TRANSFER;
    public static IntakeMotorState intakeMotorState = STOP;
    private static final PIDFController extendoPIDF = new PIDFController(0.0245,0,0.0003, 0);

    public void init() {
        setPivot(TRANSFER);
        setExtendoTarget(0);
        extendoPIDF.setTolerance(15);
        robot.colorSensor.enableLed(true);
    }

    public void autoUpdateExtendo() {
        double extendoPower = extendoPIDF.calculate(robot.extensionEncoder.getPosition(), this.target);
        extendoReached = extendoPIDF.atSetPoint();
        extendoRetracted = (target <= 0) && extendoReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0) {
            extendoPower -= 0.2;
        }

        if ((extendoReached && target > 0) || (robot.extensionEncoder.getPosition() <= 3 && target == 0)) {
            robot.extension.setPower(0);
        } else {
            robot.extension.setPower(extendoPower);
        }
    }

    public void setExtendoTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_EXTENDO_EXTENSION), 0);
        extendoPIDF.setSetPoint(this.target);
    }

    public void setPivot(IntakePivotState intakePivotState) {
        switch (intakePivotState) {
            case TRANSFER:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_TRANSFER_POS);
                break;

            case INTAKE:
                robot.leftIntakePivot.setPosition(INTAKE_PIVOT_INTAKE_POS);
                robot.rightIntakePivot.setPosition(INTAKE_PIVOT_INTAKE_POS);
                break;
        }

        Intake.intakePivotState = intakePivotState;
    }

    public void setActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakePivotState.equals(INTAKE)) {
            switch (intakeMotorState) {
                case FORWARD:
                    robot.intakeMotor.setPower(INTAKE_FORWARD_SPEED);
                    break;
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                    break;
                case STOP:
                    robot.intakeMotor.setPower(0);
                    break;
            }
            Intake.intakeMotorState = intakeMotorState;
        } else if (intakeMotorState.equals(HOLD)) {
            robot.intakeMotor.setPower(INTAKE_HOLD_SPEED);
            Intake.intakeMotorState = intakeMotorState;
        }
    }

    public void toggleActiveIntake(SampleColorTarget sampleColorTarget) {
        if (intakePivotState.equals(INTAKE)) {
            if (intakeMotorState.equals(FORWARD)) {
                setActiveIntake(STOP);
            } else if (intakeMotorState.equals(STOP) || intakeMotorState.equals(HOLD)) {
                setActiveIntake(FORWARD);
            }
            Intake.sampleColorTarget = sampleColorTarget;
        }
    }

    public void autoUpdateActiveIntake() {
        if (intakePivotState.equals(INTAKE)) {
            switch (intakeMotorState) {
                case FORWARD:
                    if (robot.colorSensor.getDistance(DistanceUnit.CM) < SAMPLE_DISTANCE_THRESHOLD) {
                        sampleColor = sampleDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                        if (correctSampleDetected()) {
                            setActiveIntake(STOP);
//                            if (opModeType.equals(OpModeType.TELEOP)) {
//                                CommandScheduler.getInstance().schedule(new realTransfer(robot.deposit, robot.intake));
//                            }
                        } else if (!sampleColor.equals(NONE)) {
                            setActiveIntake(REVERSE);
                        } else {
                            setActiveIntake(FORWARD);
                        }
                    }
                    break;
                case REVERSE:
                    if (robot.colorSensor.getDistance(DistanceUnit.CM) > SAMPLE_DISTANCE_THRESHOLD) {
                        setActiveIntake(FORWARD);
                    }
                    break;

                // No point of setting intakeMotor to 0 again
            }
        } else if (intakePivotState.equals(TRANSFER)) {
            setActiveIntake(HOLD);
        }
    }

    public static SampleColorDetected sampleDetected(int red, int green, int blue) {
        if ((blue + green + red) >= 900) {
            if (blue >= green && blue >= red) {
                return BLUE;
            } else if (green >= red) {
                return YELLOW;
            } else {
                return RED;
            }
        }
        else {
            return NONE;
        }
    }

    public static boolean correctSampleDetected() {
        switch (sampleColorTarget) {
            case ANY_COLOR:
                if (sampleColor.equals(YELLOW) ||
                   (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                    sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BLUE) ||
                    sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED)) {
                    return true;
                }
                break;
        }
        return false;
    }

    @Override
    public void periodic() {
        autoUpdateExtendo();
        autoUpdateActiveIntake();
    }
}
