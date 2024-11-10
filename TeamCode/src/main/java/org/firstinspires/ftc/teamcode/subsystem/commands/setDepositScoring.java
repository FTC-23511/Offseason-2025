package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;

public class setDepositScoring extends SequentialCommandGroup {
    public setDepositScoring(Deposit deposit, double target, Deposit.DepositPivotState state) {
        addRequirements(deposit);
        addCommands(
                new setDepositSlidesScoring(deposit, target),
                new setDeposit(deposit, state));
    }
}
