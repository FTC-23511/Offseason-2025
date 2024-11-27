package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setIntake extends CommandBase {
    private final Robot robot;
    private final Intake.IntakeMotorState motorState;
    private final Intake.IntakePivotState pivotState;
    private final double target;

    ElapsedTime timer;
    private double previousServoPos;
    private double currentServoPos;

    private boolean waitForPivot = false;

    public setIntake(Robot robot, Intake.IntakePivotState pivotState, Intake.IntakeMotorState motorState, double target) {
        this.robot = robot;
        this.pivotState = pivotState;
        this.motorState = motorState;
        this.target = target;
        this.timer = new ElapsedTime();

        addRequirements(robot.intake);
    }

    @Override
    public void initialize() {
        // Update pivot and its variables for timing below
        previousServoPos = robot.leftIntakePivot.getPosition();
        robot.intake.setPivot(pivotState);
        currentServoPos = robot.leftIntakePivot.getPosition();
        timer.reset();

        // Update motor state and extendo
        // Wait for pivot if it is reversing the motor so that sample isn't shot into the robot
        if (!motorState.equals(Intake.IntakeMotorState.REVERSE)) {
            robot.intake.setActiveIntake(motorState);
        } else {
            waitForPivot = true;
        }

        robot.intake.setExtendoTarget(target);
    }

    @Override
    public void execute() {
        if ((timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME) && motorState.equals(Intake.IntakeMotorState.REVERSE) && waitForPivot) {
            robot.intake.setActiveIntake(motorState);
            waitForPivot = false;
        }
    }

    // Command finishes when extendo has reached and pivot has had time to move
    @Override
    public boolean isFinished() {
        return robot.intake.extendoReached && (timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME);
    }
}

