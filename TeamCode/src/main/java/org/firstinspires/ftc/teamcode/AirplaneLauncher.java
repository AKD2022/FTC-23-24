package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AiprlaneLauncherTest")
public class AirplaneLauncher extends LinearOpMode {

    private Servo airplaneLauncher;
    public void runOpMode() {

        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneServo"); // 3 on control hub
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                airplaneLauncher.setPosition(1.0); // launch the plane
            } else if (gamepad1.y) {
                airplaneLauncher.setPosition(0.0); // reset the servo
            }
        }
    }
}
