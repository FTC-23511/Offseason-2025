package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositScoring extends CommandBase {
    Deposit deposit;
    private double target;
    private Deposit.DepositPivotState state;
    private boolean finished = false;
    ElapsedTime timer = new ElapsedTime();

    public setDepositScoring(Deposit deposit, double target, Deposit.DepositPivotState state) {
        this.deposit = deposit;
        this.target = target;
        this.state = state;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setClawOpen(false);
        deposit.setSlideTarget(target);
        CommandScheduler.getInstance().schedule(new setDeposit(deposit, state));
        finished = true;
    }

    @Override
    public void execute() {
        if (deposit.slidesReached && !finished) {
            deposit.setSlideTarget(target);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached && finished;
    }
}
