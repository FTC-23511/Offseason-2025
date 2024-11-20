package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class extendoSampleEject extends SequentialCommandGroup {
    public extendoSampleEject(Deposit deposit, Intake intake) {
        addCommands(
                new setDeposit(deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0),
                new InstantCommand(() -> intake.setPivot(Intake.IntakePivotState.INTAKE)),
                new setExtendo(deposit, intake, MAX_EXTENDO_EXTENSION),
                new InstantCommand(() -> intake.setActiveIntake(Intake.IntakeMotorState.REVERSE))
        );
        addRequirements(deposit, intake);
    }
}
