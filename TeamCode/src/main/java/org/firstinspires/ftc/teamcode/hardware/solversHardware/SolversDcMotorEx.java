package org.firstinspires.ftc.teamcode.hardware.solversHardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A wrapper DcMotor class that provides caching to avoid unnecessary setPower() calls.
 * Credit to team FTC 22105 (Runtime Terror) for the base class, we just modified it
 */

public class SolversDcMotorEx {
    private double lastPower = 0;
    private final DcMotorEx motor;

    private double powerThreshold = 0.0;

    public SolversDcMotorEx(DcMotorEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public SolversDcMotorEx(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    public int getPosition() {
        return motor.getCurrentPosition();
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

    public DcMotorEx.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.motor.getZeroPowerBehavior();
    }

    public DcMotorEx.RunMode getMode() {
        return this.motor.getMode();
    }

    public int getPortNumber() {
        return this.motor.getPortNumber();
    }

    public void setMode(DcMotorEx.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public double getCurrent(CurrentUnit unit) {
        return this.motor.getCurrent(unit);
    }

    public double getCurrentAlert(CurrentUnit unit) {
        return this.motor.getCurrentAlert(unit);
    }

    public void setCurrentAlert(double current, CurrentUnit unit) {
         this.motor.setCurrentAlert(current, unit);
    }

    public boolean isOverCurrent() {
        return this.motor.isOverCurrent();
    }
}