package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Auto3", group="Linear Opmode")

public class Auto3 extends LinearOpMode{
    DcMotor motor1, motor2, motor3,motor4;
    Servo servo1;
    int encoderCount = RobotPreferences.encoderCount;
    double gearReduction = RobotPreferences.gearReduction;
    double wheelSize = RobotPreferences.wheelSize;
    double wheelCircumference = RobotPreferences.wheelCircumference;
    double robotDiameter = RobotPreferences.robotDiameter;
    double robotCircumference = RobotPreferences.robotCircumference;
    double distancePerPulse = RobotPreferences.distancePerPulse;
    double pulsePerInch = RobotPreferences.pulsePerInch;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime autotime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        motor2 = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
        motor3 = hardwareMap.get(DcMotor.class, RobotPreferences.motor3);
        motor4 = hardwareMap.get(DcMotor.class, RobotPreferences.motor4);
        servo1 = hardwareMap.get(Servo.class, RobotPreferences.servo1);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        encoderDrive(1.0, 20, 20, 1000);
        encoderDrive(1.0, -robotCircumference/4, robotCircumference/4, 1000);
        encoderDrive(1.0, 135.764, 135.764, 3000);
        encoderDrive(1.0, -robotCircumference/8, robotCircumference/8, 1000);
        encoderDrive(1.0, 36, 36, 1000);
        encoderDrive(1.0, -96, -96, 1000);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        encoderStopAndResetEncoder();
        int newLeftTarget1, newLeftTarget2;
        int newRightTarget1, newRightTarget2;
        if (opModeIsActive()) {
            newLeftTarget1 = motor1.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            newLeftTarget2 = (int)(leftInches * pulsePerInch);
            newRightTarget1 = motor3.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            newRightTarget2 = (int)(leftInches * pulsePerInch);
            encoderSetTargetPosition(newLeftTarget1, newLeftTarget2, newRightTarget1, newRightTarget2);
            encoderRunToTargetPosition();
            runtime.reset();
            while((motor2.getCurrentPosition()< newLeftTarget2)&&(motor4.getCurrentPosition()<newRightTarget2)&&(motor1.getCurrentPosition()<newRightTarget1)&&(motor3.getCurrentPosition()<newLeftTarget1)){
                encoderSetPower(speed);
            }
            encoderSetPower(0);

//            while (opModeIsActive() && runtime.seconds() < timeout && motor1.isBusy() && motor3.isBusy()) {
//                telemetry.addData("Running: ", "Running at %7d: %7d", motor1.getCurrentPosition(), motor3.getCurrentPosition());
//                telemetry.update();
//            }
        }

    }
    public void encoderRunToTargetPosition(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    };

    public void encoderStopAndResetEncoder(){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    };

    public void encoderSetTargetPosition(int target1, int target2, int target3, int target4){
        motor1.setTargetPosition(target1);
        motor2.setTargetPosition(target2);
        motor3.setTargetPosition(target3);
        motor4.setTargetPosition(target4);
    };

    public void encoderSetPower(double speed){
        motor2.setPower(speed);
        motor4.setPower(speed);
        motor1.setPower(speed);
        motor3.setPower(speed);
    };

}




