package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DEPOSIT_PIVOT_MOVEMENT_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Globals.INTAKE_PIVOT_MOVEMENT_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
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
        // Update motor state and extendo
        robot.intake.setActiveIntake(motorState);
        robot.intake.setExtendoTarget(target);

        // Update pivot and its variables for timing below
        previousServoPos = robot.leftIntakePivot.getPosition();
        robot.intake.setPivot(pivotState);
        currentServoPos = robot.leftIntakePivot.getPosition();
        timer.reset();
    }

    // Command finishes when extendo has reached and pivot has had time to move
    @Override
    public boolean isFinished() {
        return robot.intake.extendoReached && (timer.milliseconds() > Math.abs(previousServoPos - currentServoPos) * INTAKE_PIVOT_MOVEMENT_TIME);
    }
}

