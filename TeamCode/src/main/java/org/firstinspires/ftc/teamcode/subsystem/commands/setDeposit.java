package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.DepositPivotState.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDeposit extends CommandBase {
    Deposit deposit;
    public Deposit.DepositPivotState state;

    ElapsedTime timer; // Timer to give claw time to close/open

    private boolean armMoved = false;
    private final double target;
    private boolean waitForClaw = false;
    private boolean armReadyToMove = false;
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
        if (state.equals(MIDDLE_HOLD) && Deposit.depositPivotState.equals(TRANSFER)) {
            deposit.setClawOpen(true);
            waitForClaw = true;
        } else if (state.equals(SPECIMEN_SCORING) && Deposit.depositPivotState.equals(INTAKE)) {
            waitForClaw = true;
        } else {
            deposit.setClawOpen(false);
        }

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
            deposit.setPivot(state);
            timer.reset();
            armMoved = true;
        } else if (armMoved && timer.milliseconds() > 500) {
            deposit.setSlideTarget(target);

            switch (state) {
                case INTAKE:
                    deposit.setClawOpen(true);
                    break;
                case TRANSFER:
                    deposit.setClawOpen(true);
                    break;
                case MIDDLE_HOLD:
                    deposit.setClawOpen(true);
                    break;
            }
            finished = true;

        } else if (deposit.slidesReached && (timer.milliseconds() > 100 || !waitForClaw)) {
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

        if (state.equals(INTAKE)) {
            deposit.setClawOpen(true);
        }
    }
}
