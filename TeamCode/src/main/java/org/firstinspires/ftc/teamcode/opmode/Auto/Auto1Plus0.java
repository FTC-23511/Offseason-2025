package org.firstinspires.ftc.teamcode.opmode.Auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.Globals.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.commands.attachSpecimen;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDeposit;

import java.util.Objects;

@Config
@Autonomous
public class Auto1Plus0 extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static int index = 0;
    public static double motorSpeeds = 0.3;
    public static int stopTimer = 2000;

    public ElapsedTime timer;

    @Override
    public void init() {
        if (Objects.equals(startingPoseName, null)) {
            throw new RuntimeException("Please set your alliance side + position in AlliancePoseSelector");
        }

        opModeType = OpModeType.AUTO;
        startingPose = STARTING_POSES.get(startingPoseName);


        CommandScheduler.getInstance().enable();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        CommandScheduler.getInstance().reset();

        robot.init(hardwareMap);

        // Initialize subsystems
        CommandScheduler.getInstance().registerSubsystem(robot.deposit, robot.intake);

    }

    @Override
    public void loop() {
        if (timer == null) {

            timer = new ElapsedTime();

            CommandScheduler.getInstance().schedule(new setDeposit(robot.deposit, Deposit.DepositPivotState.SPECIMEN_SCORING, HIGH_SPECIMEN_HEIGHT));

            robot.drive.sparkFunOTOSDrive.leftBack.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(-motorSpeeds);

            if (index == 0) {
                index = 1;
            }
        }

        telemetry.addData("index", index);
        telemetry.addData("liftBottom power", robot.liftBottom.getPower());
        telemetry.addData("liftTop power", robot.liftTop.getPower());
        telemetry.update();

        if (timer.milliseconds() >= stopTimer && index == 1) {
            robot.drive.sparkFunOTOSDrive.leftBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(0);
            index = 2;
        }

        if (timer.milliseconds() >= (3000 + stopTimer) && index == 2) {
            CommandScheduler.getInstance().schedule(new attachSpecimen(robot.deposit));

            index = 3;
        }

        if (timer.milliseconds() >= (3500 + stopTimer) && index == 3) {
            robot.deposit.setClawOpen(true);

            sleep(500);

            robot.drive.sparkFunOTOSDrive.leftBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(motorSpeeds);

            timer.reset();
            index = 4;
        }

        if (timer.milliseconds() >= (stopTimer - 300) && index == 4) {
            robot.drive.sparkFunOTOSDrive.leftBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(0);

            CommandScheduler.getInstance().schedule(new setDeposit(robot.deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0));

            if (startingPoseName.equals(PoseLocation.BLUE_BUCKET) || startingPoseName.equals(PoseLocation.RED_BUCKET)) {
                robot.drive.sparkFunOTOSDrive.leftBack.setPower(-motorSpeeds);
                robot.drive.sparkFunOTOSDrive.leftFront.setPower(+motorSpeeds);
                robot.drive.sparkFunOTOSDrive.rightBack.setPower(+motorSpeeds);
                robot.drive.sparkFunOTOSDrive.rightFront.setPower(-motorSpeeds);
            } else {
                robot.drive.sparkFunOTOSDrive.leftBack.setPower(+motorSpeeds);
                robot.drive.sparkFunOTOSDrive.leftFront.setPower(-motorSpeeds);
                robot.drive.sparkFunOTOSDrive.rightBack.setPower(-motorSpeeds);
                robot.drive.sparkFunOTOSDrive.rightFront.setPower(+motorSpeeds);
            }

            timer.reset();
            index = 5;
        }

        if (timer.milliseconds() >= stopTimer * 2 && index == 5) {
            robot.drive.sparkFunOTOSDrive.leftBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(0);
            index = 6;
        }

        CommandScheduler.getInstance().run();
    }
}