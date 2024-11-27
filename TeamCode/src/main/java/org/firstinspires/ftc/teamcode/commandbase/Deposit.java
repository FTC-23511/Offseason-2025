package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Deposit extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private static final PIDFController slidePIDF = new PIDFController(0.0125,0,0.0002, 0.00016);

    // Between open and closed
    public boolean clawOpen;

    public double target;

    public boolean slidesReached;

    // Between retracted and extended
    public boolean slidesRetracted;

    public enum DepositPivotState {
        SCORING,
        SPECIMEN_SCORING,
        TRANSFER,
        MIDDLE_HOLD,
        SPECIMEN_INTAKE
    }

    public static DepositPivotState depositPivotState;

    public void init() {
        slidePIDF.setTolerance(10, 10);
        setSlideTarget(0);

        // OpMode specific initializations
        if (opModeType.equals(OpModeType.AUTO)) {
            if (depositInit.equals(DepositInit.BUCKET_SCORING)) {
                setPivot(DepositPivotState.SCORING);
            } else {
                setPivot(DepositPivotState.SPECIMEN_SCORING);
            }
            setClawOpen(false);

        } else if (opModeType.equals(OpModeType.TELEOP)) {
            setPivot(DepositPivotState.MIDDLE_HOLD);
            setClawOpen(true);
        }
    }

    public void setSlideTarget(double target) {
        this.target = Math.max(Math.min(target, MAX_SLIDES_EXTENSION), 0);
        slidePIDF.setSetPoint(target);
    }

    public void autoUpdateSlides() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        slidesReached = slidePIDF.atSetPoint();
        slidesRetracted = (target <= 0) && slidesReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0 && !slidesReached) {
            power -= 0.1;
        } else if (target >= 2000 && !slidesReached) {
            power += 0.1;
        }

        if (slidesRetracted) {
            robot.liftTop.setPower(0);
            robot.liftBottom.setPower(0);
        } else {
            robot.liftTop.setPower(power);
            robot.liftBottom.setPower(power);
        }
    }

    public int getDepositSlidePosition() {
        return robot.liftEncoder.getPosition();
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
                break;
            case SPECIMEN_SCORING:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
                break;
            case TRANSFER:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
                break;
            case SPECIMEN_INTAKE:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
                break;
            case MIDDLE_HOLD:
                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
                break;
        }

        this.depositPivotState = depositPivotState;
    }

    public static double getDepositPivotPos() {
        switch (Deposit.depositPivotState) {
            case SPECIMEN_INTAKE:
                return DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS;
            case TRANSFER:
                return DEPOSIT_PIVOT_TRANSFER_POS;
            case SCORING:
                return DEPOSIT_PIVOT_SCORING_POS;
            case MIDDLE_HOLD:
                return DEPOSIT_PIVOT_MIDDLE_POS;
            case SPECIMEN_SCORING:
                return DEPOSIT_PIVOT_SPECIMEN_SCORING_POS;
        }
        throw new NullPointerException("depositPivotState is null!");
    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}
