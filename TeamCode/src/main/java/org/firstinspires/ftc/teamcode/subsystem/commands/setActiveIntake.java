package org.firstinspires.ftc.teamcode.subsystem.commands;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

public class setActiveIntake extends CommandBase {
    private final Intake intake;
    private final Robot robot;
    private final Intake.IntakeMotorState state;
    private boolean finished = false;

    public setActiveIntake(Intake intake, Robot robot, Intake.IntakeMotorState state) {
        this.intake = intake;
        this.robot = robot;
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

            intake.setActiveIntake(state);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setActiveIntake(Intake.IntakeMotorState.STOP);
    }
}