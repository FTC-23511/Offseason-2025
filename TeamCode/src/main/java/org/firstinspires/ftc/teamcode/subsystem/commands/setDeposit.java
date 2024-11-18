package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDeposit extends CommandBase {
    Deposit deposit;
    Deposit.DepositPivotState state;
    // Timer to give claw time to close/open
    ElapsedTime timer;
    private boolean armMoved = false;
    private double target;
    private boolean armReadyToMove = false;
    private boolean secondSlideMoveCompleted = true;
    private boolean finished = false;

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
            secondSlideMoveCompleted = false;
        }

        timer.reset();
    }

    @Override
    public void execute() {
        if (armReadyToMove) {
            deposit.setPivot(state);
            timer.reset();
            armMoved = true;

        } else if (armMoved && timer.milliseconds() > 300) {
            deposit.setClawOpen(true);
            if (!secondSlideMoveCompleted) {
                deposit.setSlideTarget(target);
            }

            switch (state) {
                case INTAKE:
                    deposit.setClawOpen(true);
                    break;
                case TRANSFER:
                    deposit.setClawOpen(false);
                    break;
                case MIDDLE_HOLD:
                    deposit.setClawOpen(true);
                    break;
            }

            finished = true;

        } else if (deposit.slidesReached) {
            armReadyToMove = true;
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }
}
