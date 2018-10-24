package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Auto1A", group="Linear Opmode")

public class Auto1A extends LinearOpMode{
    DcMotor motor1, motor2, motor3,motor4;
    Integer encoderCount = 2240;
    double gearReduction = (25 / 15);
    double wheelSize = 3.543;
    double wheelCircumference = wheelSize * Math.PI;
    double distancePerPulse;
    double pulsePerInch = 75;
    int tempVal;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        motor2 = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
        motor3 = hardwareMap.get(DcMotor.class, RobotPreferences.motor3);
        motor4 = hardwareMap.get(DcMotor.class, RobotPreferences.motor4);
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
        telemetry.addData("Running: ", "Running at %7d: %7d", motor1.getCurrentPosition(), motor3.getCurrentPosition());
        telemetry.update();

        encoderDrive(5.0, 1.0);

    }

    public void encoderDrive(double distance, double power){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tempVal = (int) (distance * pulsePerInch);
        distancePerPulse = wheelCircumference / (tempVal * gearReduction);
        telemetry.addData("Pulses", tempVal);
        motor1.setTargetPosition((int)(distancePerPulse));
        motor2.setTargetPosition((int)(distancePerPulse));
        motor3.setTargetPosition((int)(distancePerPulse));
        motor4.setTargetPosition((int)(distancePerPulse));
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
        motor4.setPower(power);
    }

//    public void motorEncoderModeSet(){
//        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

//    public void setDistanceStraight(int pulse){
//        motor1.setTargetPosition(pulse);
//        motor2.setTargetPosition(pulse);
//        motor3.setTargetPosition(pulse);
//        motor4.setTargetPosition(pulse);
//    }

//    public void runToPosition(){
//        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }



}
