package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;

public class setExtendo extends CommandBase {
    Intake intake;
    Deposit deposit;
    ElapsedTime timer;
    private double target;
    private boolean finished = false;

    public setExtendo(Deposit deposit, Intake intake, double target) {
        this.intake = intake;
        this.deposit = deposit;
        this.target = target;
        this.timer = new ElapsedTime();

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (Deposit.depositPivotState.equals(Deposit.DepositPivotState.TRANSFER)) {
            CommandScheduler.getInstance().schedule(new setDeposit(deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0));
        } else {
            intake.setExtendoTarget(target);
            finished = true;
        }
        timer.reset();
    }

    @Override
    public void execute() {
        if (!finished && timer.milliseconds() > 400) {
            intake.setExtendoTarget(target);
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return intake.extendoReached && finished;
    }
}
