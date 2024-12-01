package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class RealTransfer extends SequentialCommandGroup {
    private final Robot robot;

    public RealTransfer(Robot robot) {
        this.robot = robot;
        addCommands(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SetDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                        new SetIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0, false)
                ),
                new Transfer(robot)
        ));
    }
}
