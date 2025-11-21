package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Leave", group="Iterative OpMode")
public class AutoLeave extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    //private AprilTagLocalization aprilTagLocalization;

    @Override
    public void init() {

        //WebcamName webcamName = hardwareMap.get(WebcamName.class, "front_camera");
        //aprilTagLocalization = new AprilTagLocalization(webcamName);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void start() {
        runtime.reset();

        while (runtime.seconds() <= 1.0) {
            GreenRobot.MECANUM_DRIVE.drive(1.0, 0.0, 0.0);
        }

        GreenRobot.MECANUM_DRIVE.drive(0.0, 0.0, 0.0);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        //aprilTagLocalization.close();
    }
}
