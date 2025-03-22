package org.firstinspires.ftc.teamcode.hardware.solversHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * A wrapper DcMotor class that provides caching to avoid unnecessary setPower() calls.
 * Credit to team FTC 22105 (Runtime Terror) for the base class, we just modified it
 */

public class SolversDcMotor {
    private double lastPower = 0;
    private final DcMotor motor;

    private double powerThreshold = 0.0;

    public SolversDcMotor(DcMotor motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public SolversDcMotor(DcMotor motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    public int getPosition() {
        return(motor.getCurrentPosition());
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }
    public boolean isBusy() {
        return this.motor.isBusy();
    }

    public boolean getPowerFloat() {
        return this.motor.getPowerFloat();
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.motor.getZeroPowerBehavior();
    }

    public DcMotor.RunMode getMode() {
        return this.motor.getMode();
    }

    public int getPortNumber() {
        return this.motor.getPortNumber();
    }

    public void setMode(DcMotor.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}