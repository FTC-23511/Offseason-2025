package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private final int divideConstant = 30;
    private static final PIDFController slidePIDF = new PIDFController(0.006,0, 0.00016, 0.00032);

    // Between open and closed
    public boolean clawOpen;

    public double target;

    public boolean slidesReached;

    // Between retracted and extended
    public boolean slidesRetracted;

    public enum DepositPivotState {
        SCORING,
        SPECIMEN_SCORING,
        SPECIMEN_INTAKE,
        TRANSFER,
        MIDDLE_HOLD,
        AUTO_TOUCH_BAR
    }
    public static DepositPivotState depositPivotState;

    public void init() {
        slidePIDF.setTolerance(12, 10);
        setSlideTarget(0);

        // OpMode specific initializations
        if (opModeType.equals(OpModeType.AUTO)) {
            setPivot(depositInit);
            setClawOpen(false);
        } else if (opModeType.equals(OpModeType.TELEOP)) {
            setPivot(DepositPivotState.MIDDLE_HOLD);
            setClawOpen(true);
        }
    }

    public int getLiftScaledPosition() {
        return (robot.liftEncoder.getPosition() / divideConstant);
    }

    public void setSlideTarget(double target) {
        this.target = Range.clip(target, 0, MAX_SLIDES_EXTENSION);
        slidePIDF.setSetPoint(target);
    }

    public void autoUpdateSlides() {
        double power = slidePIDF.calculate(getLiftScaledPosition(), target);
        slidesReached = slidePIDF.atSetPoint()
                        || (getLiftScaledPosition() >= target && target == HIGH_BUCKET_HEIGHT)
                        || (target == SLIDES_PIVOT_READY_EXTENSION + 50 && getLiftScaledPosition() > SLIDES_PIVOT_READY_EXTENSION && getLiftScaledPosition() < SLIDES_PIVOT_READY_EXTENSION + 65);
        slidesRetracted = (target <= 0) && slidesReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0 && !slidesReached) {
            power -= 0.1;
        } else if (target == HIGH_SPECIMEN_ATTACH_HEIGHT && !slidesReached) {
            power += 0.3;
        }

        if (slidesRetracted) {
            robot.liftLeft.setPower(0);
            robot.liftRight.setPower(0);
        } else {
            robot.liftLeft.setPower(power);
            robot.liftRight.setPower(power);
        }
    }

    public void setClawOpen(boolean open) {
        if (open) {
            robot.depositClaw.setPosition(DEPOSIT_CLAW_OPEN_POS);
        } else {
            robot.depositClaw.setPosition(DEPOSIT_CLAW_CLOSE_POS);
        }

        this.clawOpen = open;
    }

    public void setPivot(DepositPivotState depositPivotState) {
        switch (depositPivotState) {
            case SCORING:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
                robot.depositWrist.setPosition(WRIST_SCORING);
                break;
            case SPECIMEN_SCORING:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                robot.depositWrist.setPosition(WRIST_SPECIMEN_SCORING);
                break;
            case TRANSFER:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                robot.depositWrist.setPosition(WRIST_TRANSFER);
                break;
            case SPECIMEN_INTAKE:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                robot.depositWrist.setPosition(WRIST_SPECIMEN_INTAKE);
                break;
            case MIDDLE_HOLD:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                robot.depositWrist.setPosition(WRIST_MIDDLE_HOLD);
                break;
            case AUTO_TOUCH_BAR:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_AUTO_BAR_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_AUTO_BAR_POS);
                robot.depositWrist.setPosition(WRIST_AUTO_BAR);
                break;
        }
        Deposit.depositPivotState = depositPivotState;
    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}
