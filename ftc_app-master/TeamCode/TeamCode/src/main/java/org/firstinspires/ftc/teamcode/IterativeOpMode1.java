package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="IterativeOpTest", group="Test")
public class IterativeOpMode1 extends OpMode {

    boolean flag[] = {false, false, false, false, false};
DcMotor motor1, motor2, motor3, motor4;
double pulsePerInch = 75;
    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        motor2 = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
        motor3 = hardwareMap.get(DcMotor.class, RobotPreferences.motor3);
        motor4 = hardwareMap.get(DcMotor.class, RobotPreferences.motor4);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        flag[0] = true;

    }

    @Override
    public void loop() {
        if (flag[0]) {
            encoderDrive(0.5, 6, 6);
            flag[0] = false;
        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;
        newLeftTarget = (int)(leftInches * pulsePerInch);
        newRightTarget = (int)(rightInches * pulsePerInch);
        motor2.setTargetPosition(newLeftTarget);
        motor4.setTargetPosition(newRightTarget);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ((motor2.getCurrentPosition()< newLeftTarget)&&(motor4.getCurrentPosition()< newRightTarget)){
            motor2.setPower(speed);
            motor4.setPower(speed);
        }
        motor2.setPower(0);
        motor4.setPower(0);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}