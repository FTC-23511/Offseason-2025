package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakePivotState.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.SampleColorDetected.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.SampleColorTarget.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.IntakeMotorState.*;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.commands.realTransfer;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private double target;
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
        FORWARD
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
    public static IntakePivotState intakePivotState; //  = TRANSFER
    public static IntakeMotorState intakeMotorState = STOP;
    private static final PIDFController extendoPIDF = new PIDFController(0.023,0,0, 0.001);

    public void init() {
        setPivot(TRANSFER);
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
                case REVERSE:
                    robot.intakeMotor.setPower(INTAKE_REVERSE_SPEED);
                case STOP:
                    robot.intakeMotor.setPower(0);
            }
        }

        Intake.intakeMotorState = intakeMotorState;
    }

    public void toggleActiveIntake(SampleColorTarget sampleColorTarget) {
        if (intakePivotState.equals(INTAKE)) {
            if (intakeMotorState.equals(IntakeMotorState.FORWARD)) {
                intakeMotorState = IntakeMotorState.STOP;
            } else if (intakeMotorState.equals(IntakeMotorState.STOP)) {
                intakeMotorState = IntakeMotorState.FORWARD;
            }
        }
        Intake.sampleColorTarget = sampleColorTarget;
    }

    public void autoUpdateActiveIntake(IntakeMotorState intakeMotorState) {
        if (intakePivotState.equals(INTAKE)) {
            switch (intakeMotorState) {
                case FORWARD:
                    if (robot.colorSensor.getDistance(DistanceUnit.CM) < SAMPLE_DISTANCE_THRESHOLD) {
                        sampleColor = sampleDetected(robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
                        if (correctSampleDetected()) {
                            setActiveIntake(STOP);
                            CommandScheduler.getInstance().schedule(new realTransfer(robot.deposit, robot.intake));
                        } else if (!sampleColor.equals(NONE)) {
                            setActiveIntake(REVERSE);
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
                   (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BlUE) ||
                    sampleColor.equals(RED) && allianceColor.equals(AllianceColor.RED))) {
                    return true;
                }
                break;
            case ALLIANCE_ONLY:
                if (sampleColor.equals(BLUE) && allianceColor.equals(AllianceColor.BlUE) ||
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
        autoUpdateActiveIntake(intakeMotorState);
    }
}
