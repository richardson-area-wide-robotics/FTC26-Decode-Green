package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Shoot and Intake Red", group = "Iterative OpMode", preselectTeleOp = "Default OpMode")
public class AutoShootIntakeRed extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrive mecanumDrive;
    private AprilTagLocalization aprilTagLocalization;
    private Shooter shooter;

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

    @Override
    public void start() {
        runtime.reset();
        //drive backwards for 1.25 seconds, then stop
        while (runtime.seconds() <= 1.25) {
            mecanumDrive.drive(0.0, 1.0, 0.0);
        }
        mecanumDrive.drive(0.0, 0.0, 0.0);

        // rev flywheel for 2 seconds
        runtime.reset();
        while (runtime.seconds() <= 2.0) {
            shooter.setFlywheelVelocity(1350.0);
        }

        //fire artifacts for 5 seconds, then stop
        runtime.reset();
        while (runtime.seconds() <= 5.0) {
            shooter.setFeederPower(-1.0);
            shooter.setIntakePower(1.0);
        }
        shooter.setFeederPower(0.0);
        shooter.setIntakePower(0.0);
        shooter.setFlywheelVelocity(0.0);

        //rotate clockwise for 0.2 seconds and enable intake
        runtime.reset();
        while (runtime.seconds() <= 0.2) {
            mecanumDrive.drive(0, 0, 1.0);
        }
        shooter.setIntakePower(1.0);

        //drive forward for 1.5 seconds, then stop and disable intake
        runtime.reset();
        while (runtime.seconds() <= 1.5) {
            mecanumDrive.drive(0.0, -1.0, 0.0);
            shooter.setFlywheelPower(-1.0);
        }
        mecanumDrive.drive(0, 0, 0);
        shooter.setIntakePower(0.0);

        //drive backwards for 1.25 seconds
        runtime.reset();
        while (runtime.seconds() <= 1.25) {
            mecanumDrive.drive(0.0, 1.0, 0.0);
        }

        //rotate counterclockwise for 0.2 seconds, then stop
        runtime.reset();
        while (runtime.seconds() <= 0.2) {
            mecanumDrive.drive(0, 0, -1.0);
        }
        mecanumDrive.drive(0, 0, 0);

        // rev flywheel for 2 seconds
        runtime.reset();
        while (runtime.seconds() <= 2.0) {
            shooter.setFlywheelVelocity(1350);
        }

        //fire artifacts for 5 seconds, then stop
        runtime.reset();
        while (runtime.seconds() <= 5.0) {
            shooter.setIntakePower(1.0);
            shooter.setFeederPower(-1.0);
        }
        shooter.setIntakePower(0.0);
        shooter.setFeederPower(0.0);
        shooter.setFlywheelVelocity(0.0);

        //turn counterclockwise for 0.1 seconds, then stop
        runtime.reset();
        while (runtime.seconds() <= 0.1) {
            mecanumDrive.drive(0, 0, -1.0);
        }
        mecanumDrive.drive(0, 0, 0);

        //drive forward for 0.5 seconds
        runtime.reset();
        while (runtime.seconds() <= 0.5) {
            mecanumDrive.drive(0, 1.0, 0);
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        aprilTagLocalization.close();
    }
}
