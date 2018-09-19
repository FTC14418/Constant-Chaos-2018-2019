package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoTest extends LinearOpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        rightMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);

        leftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);


    }
}
