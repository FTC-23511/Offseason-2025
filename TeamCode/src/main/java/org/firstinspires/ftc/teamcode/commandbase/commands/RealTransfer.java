package org.firstinspires.ftc.teamcode.commandbase.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.SLIDES_PIVOT_READY_EXTENSION;

import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RealTransfer extends SequentialCommandGroup {

    public RealTransfer(Robot robot) {
        addCommands(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, SLIDES_PIVOT_READY_EXTENSION + 50, true),
                        new SetIntake(robot, Intake.IntakePivotState.TRANSFER_READY, Intake.IntakeMotorState.HOLD, 0, false)
                ),
                new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, false),
                new WaitCommand(300),
                new Transfer(robot)
        ));
    }
}
