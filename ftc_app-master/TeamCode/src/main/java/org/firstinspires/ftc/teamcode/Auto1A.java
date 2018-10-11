package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto1A", group="Color1")

public class Auto1A extends LinearOpMode{
    DcMotor motor1, motor2, motor3,motor4;
    Integer encoderCount = 2240;
    double gearReduction = (25 / 15);
    double wheelSize = 3.543;
    double wheelCircumference = wheelSize * Math.PI;
    double distancePerPulse = wheelCircumference / (encoderCount * gearReduction);
    double pulsePerInch = 335;
    private ElapsedTime runtime = new ElapsedTime();
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
        encoderDrive(0.5, 5, 5, 5);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        int newLeftTarget1, newLeftTarget2;
        int newRightTarget1, newRightTarget2;
        if (opModeIsActive()) {
            newLeftTarget1 = motor1.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            newLeftTarget2 = motor2.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            newRightTarget1 = motor3.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            newRightTarget2 = motor4.getCurrentPosition() + (int)(leftInches * pulsePerInch);
            motor1.setTargetPosition(newLeftTarget1);
            motor2.setTargetPosition(newLeftTarget2);
            motor3.setTargetPosition(newRightTarget1);
            motor4.setTargetPosition(newRightTarget2);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            motor1.setPower(speed);
            motor2.setPower(speed);
            motor3.setPower(speed);
            motor4.setPower(speed);
            while (opModeIsActive() && runtime.seconds() < timeout && motor1.isBusy() && motor3.isBusy()) {
                telemetry.addData("Running: ", "Running at %7d: %7d", motor1.getCurrentPosition(), motor3.getCurrentPosition());
                telemetry.update();
            }
        }
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
