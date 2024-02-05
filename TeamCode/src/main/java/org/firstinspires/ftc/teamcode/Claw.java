package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawOnly")
public class Claw extends LinearOpMode {
    private Servo leftClaw, rightClaw, arm;
    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw"); // 0 on control hub
        rightClaw = hardwareMap.get(Servo.class, "rightClaw"); // 1 on control hub
        arm = hardwareMap.get(Servo.class, "arm"); // 2 control hub

        leftClaw.setPosition(0.35);
        rightClaw.setPosition(0.35);

        while (opModeIsActive()) {
            if (gamepad2.x) {
                leftClaw.setPosition(0.35);
                rightClaw.setPosition(0.35);
            } else if (gamepad2.y) {
                leftClaw.setPosition(0.75);
                rightClaw.setPosition(0.75);
            } else if (gamepad2.a) {
                arm.setPosition(0.75); // adjust later
            } else if (gamepad2.b) {
                arm.setPosition(0.15); // adjust later
            }

        }
    }
}
