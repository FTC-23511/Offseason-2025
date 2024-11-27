package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.DEPOSIT_PIVOT_MOVEMENT_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Globals.INTAKE_PIVOT_MOVEMENT_TIME;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class undoTransfer extends CommandBase {
    private final Robot robot;

    ElapsedTime timer;
    private int index;
    private double previousServoPos;
    private double currentServoPos;

    public undoTransfer(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();

        addRequirements(robot.intake, robot.deposit);
    }

    @Override
    public void initialize() {
        robot.deposit.setClawOpen(true);
        timer.reset();

        index = 0;
    }

    @Override
    public void execute() {
        // Wait for claw to open/let go of the sample
        if (timer.milliseconds() > 200 && index == 0) {
            // Raise the slides
            robot.deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);

            index = 1;
        }

        // Move arm
        if (robot.liftEncoder.getPosition() >= SLIDES_PIVOT_READY_EXTENSION && index == 1) {
            // Close claw so that it doesn't hit slides
            robot.deposit.setClawOpen(false);

            // Update previous and current servo pivot pos for timer logic in next if statement
            previousServoPos = robot.leftDepositPivot.getPosition();
            robot.deposit.setPivot(Deposit.DepositPivotState.MIDDLE_HOLD);
            currentServoPos = robot.leftDepositPivot.getPosition();
            timer.reset();

            index = 2;
        }

        // Move the slides back down
        if (index == 2 && timer.milliseconds() > (Math.abs(previousServoPos - currentServoPos) * DEPOSIT_PIVOT_MOVEMENT_TIME)) {
            robot.deposit.setSlideTarget(0);
            robot.deposit.setClawOpen(true);

            index = 3;
        }
    }

    // Command finishes when all execute commands have finished (index == 3), and slides have retracted
    @Override
    public boolean isFinished() {
        return robot.deposit.slidesReached && index == 3;
    }
}

