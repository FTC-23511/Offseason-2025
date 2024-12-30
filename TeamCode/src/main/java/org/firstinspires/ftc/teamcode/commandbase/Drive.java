package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;


public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum HangState {
        HANG_RETRACTED,
        HANG_EXTENDED
    }

    public static HangState hangState;

    public void init() {
        setHang(HangState.HANG_RETRACTED);
    }

    public void setHang(HangState hangState) {
        switch (hangState) {
            case HANG_RETRACTED:
                robot.leftHang.setPosition(HANG_RETRACTED_POS);
                robot.rightHang.setPosition(HANG_RETRACTED_POS);
                break;
            case HANG_EXTENDED:
                robot.leftHang.setPosition(HANG_EXTENDED_POS);
                robot.rightHang.setPosition(HANG_EXTENDED_POS);
                break;
        }

        Drive.hangState = hangState;
    }
}
