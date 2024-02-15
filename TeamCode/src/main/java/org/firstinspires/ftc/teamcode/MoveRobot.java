package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MoveRobot {
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    public MoveRobot(DcMotor motorFrontLeft, DcMotor motorFrontRight, DcMotor motorBackLeft, DcMotor motorBackRight) {
        this.motorFrontLeft = motorFrontLeft;
        this.motorFrontRight = motorFrontRight;
        this.motorBackLeft = motorBackLeft;
        this.motorBackRight = motorBackRight;
    }

    public void moveRobot(double drive, double strafe, double rotate) {
        double powerFrontLeft = drive + strafe + rotate;
        double powerFrontRight = drive - strafe - rotate;
        double powerBackLeft = drive - strafe + rotate;
        double powerBackRight = drive + strafe - rotate;

        // Normalize the powers so that no motor exceeds 1.0
        double maxPower = Math.max(Math.max(Math.abs(powerFrontLeft), Math.abs(powerFrontRight)),
                Math.max(Math.abs(powerBackLeft), Math.abs(powerBackRight)));
        if (maxPower > 1.0) {
            powerFrontLeft /= maxPower;
            powerFrontRight /= maxPower;
            powerBackLeft /= maxPower;
            powerBackRight /= maxPower;
        }

        motorFrontLeft.setPower(powerFrontLeft);
        motorFrontRight.setPower(powerFrontRight);
        motorBackLeft.setPower(powerBackLeft);
        motorBackRight.setPower(powerBackRight);
    }
}