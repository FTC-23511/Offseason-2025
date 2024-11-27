package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setDeposit extends CommandBase {
    private final Robot robot;
    private final Deposit.DepositPivotState state;
    private final double target;
    private final boolean clawOpen;

    ElapsedTime timer;
    private boolean armReadyToMove = false;
    private boolean armMoved = false;
    private boolean finished = false;
    private double previousServoPos;
    private double currentServoPos;

    public setDeposit(Robot robot, Deposit.DepositPivotState state, double target, boolean clawOpen) {
        this.robot = robot;
        this.state = state;
        this.target = target;
        this.clawOpen = clawOpen;
        this.timer = new ElapsedTime();

        addRequirements(robot.deposit);
    }

    @Override
    public void initialize() {
        // Always close claw first in case of any arm movements that need to be done
        robot.deposit.setClawOpen(false);

        // Move slides to above pivot ready extension if target is below the pivot ready extension so that arm can move later
        // If it is more than that just yolo it because slides are faster than the pivot so arm is ready to move instantly
        if (target >= SLIDES_PIVOT_READY_EXTENSION) {
            robot.deposit.setSlideTarget(target);
            armReadyToMove = true;
        } else {
            robot.deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);
        }
        timer.reset();
    }

    @Override
    public void execute() {
        // Should run if the target is less than SLIDES_PIVOT_READY_EXTENSION (due to the if statement above) AND when the slides are actually above that height
        // Should run first or never depending on target in loop
        if (robot.liftEncoder.getPosition() >= SLIDES_PIVOT_READY_EXTENSION && target >= SLIDES_PIVOT_READY_EXTENSION && !armReadyToMove) {
            armReadyToMove = true;
        }

        // Should run in the loop immediately after armReadyToMove = true and move the arm
        if (armReadyToMove && !armMoved) {
            // Update previous and current servo pivot pos for timer logic in next if statement
            previousServoPos = robot.leftDepositPivot.getPosition();
            robot.deposit.setPivot(state);
            currentServoPos = robot.leftDepositPivot.getPosition();
            timer.reset();

            armMoved = true;
        }

        // Should be final move of claw and slides to the original desired state after arm has moved
        // Uses difference in pos of claw multiplied by a constant that converts servoPos change to milliseconds (DEPOSIT_PIVOT_MOVEMENT_TIME)
        if (armMoved && timer.milliseconds() > (Math.abs(previousServoPos - currentServoPos) * DEPOSIT_PIVOT_MOVEMENT_TIME)) {
            robot.deposit.setSlideTarget(target);
            robot.deposit.setClawOpen(clawOpen);
            finished = true;
        }
    }

    // Command finishes when slides have reached and all arm movements are finished
    @Override
    public boolean isFinished() {
        return robot.deposit.slidesReached && finished;
    }
}

