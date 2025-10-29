/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="Default OpMode (31585)", group="Iterative OpMode")
public class DefaultOpMode extends OpMode
{
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private AprilTagLocalization aprilTagLocalization;
    private Shooter shooter;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotorEx frontLeftDrive = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        DcMotorEx backLeftDrive = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        DcMotorEx frontRightDrive = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        DcMotorEx backRightDrive = hardwareMap.get(DcMotorEx.class, "back_right_drive");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        mecanumDrive = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, imu);

        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        DcMotorEx feederMotor = hardwareMap.get(DcMotorEx.class, "feeder_motor");
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");

        shooter = new Shooter(flywheelMotor, feederMotor, intakeMotor);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "front_camera");
        aprilTagLocalization = new AprilTagLocalization(webcamName);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // Hold LEFT TRIGGER to unload shooter
        if (gamepad1.left_trigger > 0.0) {
            shooter.setFlywheelVelocity(-825.0);
        } else {
            shooter.setFlywheelVelocity(0.0);
        }

        // Hold RIGHT TRIGGER to shoot
        if (gamepad1.right_trigger > 0.0) {
            shooter.setFlywheelVelocity(1750.0);
        } else {
            shooter.setFlywheelVelocity(0.0);
        }

        if (gamepad1.left_bumper) {
            shooter.setFeederVelocity(500.0);
        } else {
            shooter.setFeederVelocity(0.0);
        }

        if (gamepad1.right_bumper) {
            shooter.setFeederVelocity(-500.0);
        } else {
            shooter.setFeederVelocity(0.0);
        }

        if (gamepad1.b) {
            shooter.setIntakeVelocity(750.0);
        } else {
            shooter.setIntakeVelocity(0.0);
        }

        // Press A to reset the robot heading
        if (gamepad1.a) {
            mecanumDrive.resetYaw();
        }

        // Hold Y to drive robot-oriented instead of field-oriented
        // Use LEFT STICK to strafe in any direction and RIGHT STICK to rotate
        if (gamepad1.y) {
            mecanumDrive.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            mecanumDrive.driveFieldRelative(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        aprilTagLocalization.telemetryAprilTag(telemetry);
        telemetry.addData("Flywheel Velocity", shooter.getFlywheelVelocity());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        aprilTagLocalization.close();
    }

}
