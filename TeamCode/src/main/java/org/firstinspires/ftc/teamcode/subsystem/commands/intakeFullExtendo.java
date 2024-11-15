package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.*;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class intakeFullExtendo extends ParallelCommandGroup {
    public intakeFullExtendo(Intake intake) {
        addCommands(new InstantCommand(() -> intake.setPivot(IntakePivotState.READY_INTAKE)),
                    new InstantCommand(() -> intake.setExtendoTarget(MAX_EXTENDO_EXTENSION)));
        addRequirements(intake);
    }
}
