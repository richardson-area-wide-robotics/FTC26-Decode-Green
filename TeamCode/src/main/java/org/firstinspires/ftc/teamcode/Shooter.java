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
        feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        feederMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor.setVelocityPIDFCoefficients(900.0, 0.25, 0.0, 0.0);
        feederMotor.setVelocityPIDFCoefficients(50.0, 0.001, 0.0, 0.0);
    }

    public void setFlywheelVelocity(double velocity) {
        flywheelMotor.setVelocity(velocity);
    }

    public void setFeederVelocity(double velocity) {
        feederMotor.setVelocity(velocity);
    }

    public void setIntakeVelocity(double velocity) {
        intakeMotor.setVelocity(velocity);
    }

    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }

}
