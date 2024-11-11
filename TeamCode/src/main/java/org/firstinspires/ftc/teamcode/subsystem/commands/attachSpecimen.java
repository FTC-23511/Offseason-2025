package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class attachSpecimen extends CommandBase {
    Deposit deposit;
    private boolean finished = false;
    ElapsedTime timer = new ElapsedTime();

    public attachSpecimen(Deposit deposit) {
        this.deposit = deposit;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setSlideTarget(HIGH_SPECIMEN_ATTACH_HEIGHT);
    }

    @Override
    public boolean isFinished() {
        return deposit.slidesReached;
    }

    @Override
    public void end(boolean interruptable) {
        deposit.setClawOpen(true);
    }
}
