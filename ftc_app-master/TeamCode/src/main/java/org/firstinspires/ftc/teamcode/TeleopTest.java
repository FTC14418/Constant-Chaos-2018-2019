package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpTest", group="Iterative Opmode")
public class TeleopTest extends OpMode {

    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;
    Servo servo;

    @Override
    public void init() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        leftBackMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor3);
        rightBackMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor4);
        servo = hardwareMap.get(Servo.class, RobotPreferences.servo1);
    }

    @Override
    public void loop(){
         arcadeDrive(-gamepad1.right_stick_x, gamepad1.left_stick_y);
         if(gamepad1.a){
             servo(0);
         }
         else if(gamepad1.b){
             servo(90);
         }
        telemetry.addData("Running: ", "Running at %7d: %7d: %7d: %7d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition(), leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
        telemetry.update();

    }
    public void arcadeDrive(double drive , double turn){
        double leftPower = Range.clip(drive + turn, -1.0, 1.0 );
        double rightPower = Range.clip(drive-turn,-1.0, 1.0);
        leftFrontMotor.setPower(leftPower);
        leftBackMotor.setPower(leftPower);
        rightFrontMotor.setPower(rightPower);
        rightBackMotor.setPower(rightPower);

    }

    public void servo(int position){
        servo.setPosition(position);
    }

}