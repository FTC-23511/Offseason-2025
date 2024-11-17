package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class transferReady extends SequentialCommandGroup {
    public transferReady(Deposit deposit, Intake intake) {
        addCommands(
                new InstantCommand(() -> intake.setPivot(Intake.IntakePivotState.TRANSFER)),
                new InstantCommand(() -> intake.setExtendoTarget(0)),
                new setDeposit(deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0)
        );
        addRequirements(deposit, intake);
    }
}
