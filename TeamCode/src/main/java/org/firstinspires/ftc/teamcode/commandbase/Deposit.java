package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.qualcomm.robotcore.util.Range;

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

    public enum DepositWristState {
        SCORING,
        FRONT_SPECIMEN_SCORING,
        BACK_SPECIMEN_SCORING,
        FRONT_SPECIMEN_INTAKE,
        BACK_SPECIMEN_INTAKE,
        TRANSFER,
        MIDDLE_HOLD,
    }

    public static DepositPivotState depositPivotState;
    public static DepositWristState depositWristState;

    public void init() {
        slidePIDF.setTolerance(12, 10);
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
        this.target = Range.clip(target, 0, MAX_SLIDES_EXTENSION);
        slidePIDF.setSetPoint(target);
    }

    public void autoUpdateSlides() {
        double power = slidePIDF.calculate(robot.liftEncoder.getPosition(), target);
        slidesReached = slidePIDF.atSetPoint() || (robot.liftEncoder.getPosition() >= target && target == HIGH_BUCKET_HEIGHT);
        slidesRetracted = (target <= 0) && slidesReached;

        // Just make sure it gets to fully retracted if target is 0
        if (target == 0 && !slidesReached) {
            power -= 0.1;
        } else if (target >= HIGH_BUCKET_HEIGHT && !slidesReached) {
            power += 0.6;
        }

        if (slidesRetracted) {
            robot.liftTop.setPower(0);
            robot.liftBottom.setPower(0);
        } else {
            robot.liftTop.setPower(power);
            robot.liftBottom.setPower(power);
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
//        switch (depositPivotState) {
//            case SCORING:
//                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
//                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SCORING_POS);
//                break;
//            case SPECIMEN_SCORING:
//                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_FRONT_INTAKE_POS);
//                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_SCORING_POS);
//                break;
//            case TRANSFER:
//                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
//                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_TRANSFER_POS);
//                break;
//            case SPECIMEN_INTAKE:
//                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
//                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_SPECIMEN_INTAKE_POS);
//                break;
//            case MIDDLE_HOLD:
//                robot.leftDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
//                robot.rightDepositPivot.setPosition(DEPOSIT_PIVOT_MIDDLE_POS);
//                break;
//        }

        Deposit.depositPivotState = depositPivotState;
    }

    public void setWrist(DepositWristState depositWristState) {
        switch (depositWristState) {
            case SCORING:
                robot.depositWrist.setPosition(WRIST_SCORING);
                break;
            case FRONT_SPECIMEN_SCORING:
                robot.depositWrist.setPosition(WRIST_FRONT_SPECIMEN_SCORING);
                break;
            case BACK_SPECIMEN_SCORING:
                robot.depositWrist.setPosition(WRIST_BACK_SPECIMEN_SCORING);
                break;
            case FRONT_SPECIMEN_INTAKE:
                robot.depositWrist.setPosition(WRIST_FRONT_SPECIMEN_INTAKE);
                break;
            case BACK_SPECIMEN_INTAKE:
                robot.depositWrist.setPosition(WRIST_BACK_SPECIMEN_INTAKE);
                break;
            case TRANSFER:
                robot.depositWrist.setPosition(WRIST_TRANSFER);
                break;
            case MIDDLE_HOLD:
                robot.depositWrist.setPosition(WRIST_MIDDLE_HOLD);
                break;
        }

        Deposit.depositWristState = depositWristState;
    }

    @Override
    public void periodic() {
        autoUpdateSlides();
    }
}
