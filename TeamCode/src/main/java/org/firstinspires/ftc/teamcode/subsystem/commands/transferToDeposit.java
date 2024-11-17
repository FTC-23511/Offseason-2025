package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferToDeposit extends SequentialCommandGroup {
    public transferToDeposit(Deposit deposit, Intake intake) {
        addCommands(
                new setDeposit(deposit, Deposit.DepositPivotState.TRANSFER, 0),
                new InstantCommand(() -> deposit.setClawOpen(false))
        );
        addRequirements(deposit, intake);
    }
}
