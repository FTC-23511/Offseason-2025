package org.firstinspires.ftc.teamcode.tuning.motor;

import static org.firstinspires.ftc.teamcode.hardware.Globals.OpModeType;
import static org.firstinspires.ftc.teamcode.hardware.Globals.opModeType;
import static org.firstinspires.ftc.teamcode.hardware.SolversMethods.checkButton;
import static org.firstinspires.ftc.teamcode.tuning.example.ExampleConstants.CENTER_MOTOR_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.solversHardware.SolversMotor;

//@Photon
@Config
@TeleOp
public class RawMotorTester extends OpMode {
    public static boolean USE_DASHBOARD = false;
    public static double power = 0;
    public static String name = "extension";

    Gamepad currentGamepad1 = new Gamepad();
    private SolversMotor motor;

    @Override
    public void init() {
        opModeType = OpModeType.TELEOP;

        motor = new SolversMotor(hardwareMap.get(DcMotor.class, name));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(CENTER_MOTOR_POWER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("Motor Encoder", motor.getPosition());
    }

    @Override
    public void loop() {
        if (USE_DASHBOARD){
            motor.setPower(power);
        } else if (gamepad1.dpad_up  && checkButton(currentGamepad1, "dpad_up")) {
            power += 0.01;
        } else if (gamepad1.dpad_down && checkButton(currentGamepad1, "dpad_down")) {
            power -= 0.01;
        }

        if (gamepad1.square || gamepad1.triangle || gamepad1.circle || gamepad1.cross) {
            motor.setPower(CENTER_MOTOR_POWER);
        }
        else{
            motor.setPower(0);
        }

        currentGamepad1.copy(gamepad1);

        telemetry.addData("centerMotor Power", motor.getPower());
        telemetry.addData("centerMotor Encoder", motor.getPosition());
        telemetry.update();
    }
}