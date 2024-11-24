package org.firstinspires.ftc.teamcode.subsystem.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class waitForIntakeCompletion extends CommandBase {
    Intake intake;

    public waitForIntakeCompletion(Intake intake) {
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public boolean isFinished() {
        return Intake.correctSampleDetected();
    }
}
