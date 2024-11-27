package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.commandbase.Deposit.getDepositPivotPos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;

public class setDeposit extends CommandBase {
    Deposit deposit;
    public Deposit.DepositPivotState state;

    ElapsedTime timer;
    private boolean armReadyToMove = false;
    private boolean armMoved = false;
    private final double target;
    private boolean finished = false;
    private double previousServoPos;
    private double currentServoPos;

    public setDeposit(Deposit deposit, Deposit.DepositPivotState state, double target) {
        this.deposit = deposit;
        this.state = state;
        this.target = target;
        this.timer = new ElapsedTime();

        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setClawOpen(false);

        if (target >= SLIDES_PIVOT_READY_EXTENSION) {
            deposit.setSlideTarget(target);
            armReadyToMove = true;
        } else {
            deposit.setSlideTarget(SLIDES_PIVOT_READY_EXTENSION + 50);
        }
        timer.reset();
    }

    @Override
    public void execute() {
        if (armReadyToMove && !armMoved) {
            previousServoPos = getDepositPivotPos();

            deposit.setPivot(state);

            currentServoPos = getDepositPivotPos();

            timer.reset();
            armMoved = true;
        } else if (armMoved && timer.milliseconds() > (Math.abs(previousServoPos - currentServoPos) * DEPOSIT_PIVOT_MOVEMENT_TIME)) {
            deposit.setSlideTarget(target);
            switch (state) {
                case TRANSFER: case MIDDLE_HOLD:
                    deposit.setClawOpen(true);
                    break;
            }
            finished = true;

        } else if (deposit.slidesReached && (timer.milliseconds() > 200)) {
            armReadyToMove = true;
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }

    @Override
    public void end(boolean interruptable) {
        deposit.setPivot(state);
        deposit.setSlideTarget(target);
    }
}

