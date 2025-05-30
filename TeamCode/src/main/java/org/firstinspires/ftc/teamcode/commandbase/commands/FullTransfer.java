package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class FullTransfer extends SequentialCommandGroup {
    public FullTransfer(Robot robot) {
        addCommands(
                new ParallelCommandGroup(
                        new SetDeposit(robot, Deposit.DepositPivotState.TRANSFER, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, false)
                ),
                new InstantCommand(() -> Intake.transferring = true),
                new ServoOnlyTransfer(robot)
        );
    }

    @Override
    public void end(boolean interrupted) {
        Intake.transferring = false;
        super.end(interrupted);
    }
}
