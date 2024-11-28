package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Transfer extends SequentialCommandGroup {
    public Transfer(Robot robot) {
        addCommands(
                new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, 0, true),
                new InstantCommand(() -> robot.deposit.setClawOpen(false))
        );
        addRequirements(robot.intake, robot.deposit);
    }
}
