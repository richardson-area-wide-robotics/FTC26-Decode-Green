package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Shooter {
    private final DcMotorEx flywheelMotor;
    private final DcMotorEx feederMotor;
    private final DcMotorEx intakeMotor;

    public Shooter(DcMotorEx flywheelMotor, DcMotorEx feederMotor, DcMotorEx intakeMotor) {
        this.flywheelMotor = flywheelMotor;
        this.feederMotor = feederMotor;
        this.intakeMotor = intakeMotor;

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        feederMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // This is incredibly stupid but it works for some reason
        flywheelMotor.setVelocityPIDFCoefficients(500.0, 0.0, 0.0, 0.0);
    }

    public void setFlywheelVelocity(double velocity) {
        flywheelMotor.setVelocity(velocity);
    }

    public void setFlywheelPower(double power) {
        flywheelMotor.setPower(power);
    }

    public void setFeederPower(double power) {
        feederMotor.setPower(power);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }

    public double getFeederVelocity() {
        return feederMotor.getVelocity();
    }

    public double getIntakeVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

}
