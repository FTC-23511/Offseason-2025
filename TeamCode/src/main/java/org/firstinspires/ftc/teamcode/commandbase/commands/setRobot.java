package org.firstinspires.ftc.teamcode.commandbase.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commandbase.Deposit;
import org.firstinspires.ftc.teamcode.commandbase.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class setRobot extends ParallelCommandGroup {
    private final Robot robot;
    private final Robot.RobotState robotState;
    private boolean didItReallyHappen = false;
    public setRobot(Robot robot, Robot.RobotState robotState) {
        this.robot = robot;
        this.robotState = robotState;

        switch (robotState) {
            case RESTING:
                if (!Robot.robotState.equals(Robot.RobotState.TRANSFERRED)) {
                    addCommands(
                            new setDeposit(robot, Deposit.DepositPivotState.MIDDLE_HOLD, 0, true),
                            new setIntake(robot, Intake.IntakePivotState.TRANSFER, Intake.IntakeMotorState.HOLD, 0)
                    );
                    addRequirements(robot.intake, robot.deposit);
                } else {
                    addCommands(
                            new undoTransfer(robot)
                    );
                    addRequirements(robot.intake, robot.deposit);
                }

                didItReallyHappen = true;
                break;

            case TRANSFERRED:
                if (Robot.robotState.equals(Robot.RobotState.RESTING)) {
                    addCommands(
                            new transfer(robot)
                    );
                    addRequirements(robot.intake, robot.deposit);

                    didItReallyHappen = true;
                }
                break;
            case SPECIMEN_SCORING:
                break;
            case SPECIMEN_INTAKING:
                break;
        }
    }

    @Override
    public void end(boolean interruptible) {
        if (didItReallyHappen) {
            Robot.robotState = robotState;
        }
    }


}
