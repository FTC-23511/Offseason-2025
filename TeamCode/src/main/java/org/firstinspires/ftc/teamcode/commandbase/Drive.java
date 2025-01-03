package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum HangState {
        RETRACT_HANG,
        EXTEND_HANG,
        STOP_HANG
    }

    public enum GearboxState {
        HANG_GEAR,
        DEPOSIT_GEAR
    }

    public static HangState hangState;
    public static GearboxState gearboxState;

    public void init() {
        setHang(HangState.RETRACT_HANG);
        setGearbox(GearboxState.DEPOSIT_GEAR);
    }

    public void setHang(HangState hangState) {
        switch (hangState) {
            case RETRACT_HANG:
                robot.leftHang.setPower(-LEFT_HANG_FULL_POWER);
                robot.rightHang.setPower(RIGHT_HANG_FULL_POWER);
                break;
            case EXTEND_HANG:
                robot.leftHang.setPower(LEFT_HANG_FULL_POWER);
                robot.rightHang.setPower(RIGHT_HANG_FULL_POWER);
                break;
            case STOP_HANG:
                robot.leftHang.setPower(0);
                robot.rightHang.setPower(0);
                break;
        }

        Drive.hangState = hangState;
    }

    public void setGearbox(GearboxState gearboxState) {
        switch (gearboxState) {
            case HANG_GEAR:
                robot.gearboxSwitcher.setPosition(HANG_GEAR_POS);
                break;
            case DEPOSIT_GEAR:
                robot.gearboxSwitcher.setPosition(DEPOSIT_GEAR_POS);
                break;
        }

        Drive.gearboxState = gearboxState;
    }
}
