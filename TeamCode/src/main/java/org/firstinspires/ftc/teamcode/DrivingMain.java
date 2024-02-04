package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Arnesh, Owen
@TeleOp(name="MainOpMode22280")
public class DrivingMain extends LinearOpMode  {
    private DcMotor fL, fR, bL, bR;
    private GamepadEx driverController1;
    private GamepadEx driverController2;

    private Motor lLift, rLift;
    private MotorGroup lift;
    private boolean maxLiftSpeed = false;

    private ServoEx rightArmServo;
    private Servo airplaneLauncher;
    private ServoEx leftClawServo, rightClawServo;

    private AirplaneLauncher ap;

    /*
    public void closeClaw(){
        leftClawServo.setPosition(0.35);
        rightClawServo.setPosition(0.35);
    }

    public void openClaw(){
        leftClawServo.setPosition(0.65);
        rightClawServo.setPosition(0.65);
    }

    public void positionArmToScore(){
        rightArmServo.setPosition(.5);
    }

    public void positionArmToRest(){
        rightArmServo.setPosition(.1);
    }
    */
    @Override
    public void runOpMode() throws InterruptedException{
        fL = hardwareMap.get(DcMotor.class, "fL"); //port 0
        fR = hardwareMap.get(DcMotor.class, "fR"); //port 1
        bL = hardwareMap.get(DcMotor.class, "bL"); //port 2
        bR = hardwareMap.get(DcMotor.class, "bR"); //port 3
        
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneServo");




        /*
        lLift = new Motor(hardwareMap, "lLift", Motor.GoBILDA.RPM_223);
        rLift = new Motor(hardwareMap, "rLift", Motor.GoBILDA.RPM_223);

        lLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        lLift.setInverted(true);

        rLift.resetEncoder();
        lLift.resetEncoder();

        lift = new MotorGroup(lLift, rLift);

        lift.setRunMode(Motor.RunMode.PositionControl);
        lift.setPositionCoefficient(LIFT_POSITION_COEFFICIENT);
        lift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        int targetLiftPosition = lLift.getCurrentPosition();
        */

        /*
        rightArmServo = new SimpleServo(hardwareMap, "rightArmServo", 0, 360, AngleUnit.DEGREES);

        rightClawServo = new SimpleServo(hardwareMap, "rightClawServo", 0, 360, AngleUnit.DEGREES);
        leftClawServo = new SimpleServo(hardwareMap, "leftClawServo", 0, 360, AngleUnit.DEGREES);

        leftClawServo.setInverted(true);
        closeClaw();
         */

        waitForStart();
        while(opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);

            // Update telemetry
            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();


            if (gamepad1.x) {
                airplaneLauncher.setPosition(1.0);
            } else if (gamepad1.y) {
                airplaneLauncher.setPosition(0);
            }
        }




            /*
            if (gamepad2.right_bumper){
                targetLiftPosition = 5000;
                maxLiftSpeed = true;
            }else if (gamepad2.left_bumper) {
                targetLiftPosition = 0;
                maxLiftSpeed = true;
            }
            else if (gamepad2.right_trigger > 0){
                targetLiftPosition += gamepad2.right_trigger * 25;
                maxLiftSpeed = false;
            } else if (gamepad2.left_trigger > 0) {
                targetLiftPosition -= gamepad2.left_trigger * 25;
                maxLiftSpeed = false;
            }

            if (targetLiftPosition < 0){
                targetLiftPosition = 0;
                maxLiftSpeed = false;
            }
            if (targetLiftPosition > 6000){
                targetLiftPosition = 5700;
                maxLiftSpeed = false;
            }

            lift.setTargetPosition(targetLiftPosition);

            if(lift.atTargetPosition()){
                lift.set(0); // same as .set(0)??? test this!
                maxLiftSpeed = false;
            }else{
                if(maxLiftSpeed){
                    lift.set(1);
                }
                else if (lLift.getCurrentPosition() <= 700 || lLift.getCurrentPosition() >= 2300){
                    lift.set(0.2);
                }else{
                    lift.set(0.85);
                }
            }

            /*
            if(targetLiftPosition < 2000){
                positionArmToRest();
                closeClaw();
            }else if(gamepad2.a){
                positionArmToRest();
            }else if(gamepad2.y){
                positionArmToScore();
            }

            if(lLift.getCurrentPosition() > 4000){
                positionArmToScore();
            }

            if(gamepad2.x){
                closeClaw();
            }else if(gamepad2.b){
                openClaw();
            }
             */


            telemetry.update();




        }
}