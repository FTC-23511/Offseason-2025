package org.firstinspires.ftc.teamcode.commandbase;

import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Drive extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum OctocanumServosState {
        RETRACTED,
        EXTENDED,
        PTO_AND_RETRACTED
    }

    public static OctocanumServosState octocanumServosState;

    public void init() {
        setOctocanumServos(OctocanumServosState.RETRACTED);
    }

    public void setOctocanumServos(OctocanumServosState state) {
        switch (state) {
            case RETRACTED:
                robot.FR.setPosition(FR_RETRACTED);
                robot.FL.setPosition(FL_RETRACTED);
                robot.BR.setPosition(BR_RETRACTED);
                robot.BL.setPosition(BL_RETRACTED);
                break;
            case EXTENDED:
                robot.FR.setPosition(FR_EXTENDED);
                robot.FL.setPosition(FL_EXTENDED);
                robot.BR.setPosition(BR_EXTENDED);
                robot.BL.setPosition(BL_EXTENDED);
                break;
            case PTO_AND_RETRACTED:
                robot.FR.setPosition(FR_PTO_RETRACTED);
                robot.FL.setPosition(FL_PTO_RETRACTED);
                robot.BR.setPosition(BR_PTO_RETRACTED);
                robot.BL.setPosition(BL_PTO_RETRACTED);
                break;
        }

        octocanumServosState = state;
    }
}
