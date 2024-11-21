package org.firstinspires.ftc.teamcode.opmode.Auto;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_BUCKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.HIGH_SPECIMEN_HEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.PoseLocation;
import static org.firstinspires.ftc.teamcode.hardware.Globals.STARTING_POSES;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.startingPose;
import static org.firstinspires.ftc.teamcode.hardware.Globals.startingPoseName;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.commands.attachSpecimen;
import org.firstinspires.ftc.teamcode.subsystem.commands.setDeposit;

import java.util.Objects;

@Config
@Autonomous
public class Auto0Plus1 extends OpMode {
    private final Robot robot = Robot.getInstance();
    public static int index = 0;
    public static double motorSpeeds = 0.3;
    public static int stopTimer = 500;

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

            CommandScheduler.getInstance().schedule(new setDeposit(robot.deposit, Deposit.DepositPivotState.SCORING, HIGH_BUCKET_HEIGHT));
            sleep(2000);
            robot.drive.sparkFunOTOSDrive.leftBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(motorSpeeds);

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

        if (timer.milliseconds() >= (1000 + stopTimer) && index == 2) {
            robot.deposit.setClawOpen(true);

            sleep(500);

            robot.drive.sparkFunOTOSDrive.leftBack.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(-motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(-motorSpeeds);

            timer.reset();
            index = 3;
        }

        if (timer.milliseconds() >= (1000) && index == 3) {
            robot.drive.sparkFunOTOSDrive.leftBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(0);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(0);

            CommandScheduler.getInstance().schedule(new setDeposit(robot.deposit, Deposit.DepositPivotState.MIDDLE_HOLD, 0));

            sleep(500);

            robot.drive.sparkFunOTOSDrive.leftBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.leftFront.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightBack.setPower(motorSpeeds);
            robot.drive.sparkFunOTOSDrive.rightFront.setPower(motorSpeeds);

            timer.reset();
            index = 4;
        }

        CommandScheduler.getInstance().run();
    }
}