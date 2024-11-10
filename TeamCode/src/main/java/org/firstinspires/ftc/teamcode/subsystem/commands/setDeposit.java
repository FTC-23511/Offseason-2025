package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDeposit extends CommandBase {
    Deposit deposit;
    Deposit.DepositPivotState state;
    // Timer to give claw time to close/open
    ElapsedTime timer = new ElapsedTime();
    private boolean finished = false;

    public setDeposit(Deposit deposit, Deposit.DepositPivotState state) {
        this.deposit = deposit;
        this.state = state;

        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        switch (state) {
            case MIDDLE_HOLD:
                deposit.setClawOpen(true);
                break;
            case SCORING:
                deposit.setClawOpen(false);
                break;
            case SPECIMEN_SCORING:
                deposit.setClawOpen(false);
                break;
            case TRANSFER:
                deposit.setClawOpen(true);
                break;
        }
        timer.reset();
    }

    @Override
    public void execute() {
        if ((deposit.getDepositSlidePosition() >= SLIDES_PIVOT_READY_EXTENSION) && (deposit.target >= SLIDES_PIVOT_READY_EXTENSION) && !finished) {
            deposit.setPivot(state);
            timer.reset();
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (timer.milliseconds() > 300) && finished;
    }
}
