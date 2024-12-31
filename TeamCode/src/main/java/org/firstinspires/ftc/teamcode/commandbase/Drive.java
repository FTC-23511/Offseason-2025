package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum HangState {
        HANG_RETRACTED,
        HANG_EXTENDED,
        HANG_SWINGING
    }

    public enum GearboxState {
        HANG_GEAR,
        DEPOSIT_GEAR
    }

    public static HangState hangState;
    public static GearboxState gearboxState;

    public void init() {
        setHang(HangState.HANG_RETRACTED);
        setGearbox(GearboxState.DEPOSIT_GEAR);
    }

    public void setHang(HangState hangState) {
        switch (hangState) {
            case HANG_RETRACTED:
                robot.leftHang.setPower(HANG_RETRACTED_POS);
                robot.rightHang.setPower(HANG_RETRACTED_POS);
                break;
            case HANG_EXTENDED:
                robot.leftHang.setPower(HANG_EXTENDED_POS);
                robot.rightHang.setPower(HANG_EXTENDED_POS);
                break;
            case HANG_SWINGING:
                robot.leftHang.setPower(HANG_SWINGING_POS);
                robot.rightHang.setPower(HANG_SWINGING_POS);
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
