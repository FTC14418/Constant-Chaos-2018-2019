package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="AutoTest", group="Linear Opmode")
public class AutoTest extends LinearOpMode{
    DcMotor leftMotor;
    DcMotor rightMotor;
    DistanceSensor distanceSensor1;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        rightMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, RobotPreferences.distanceSensor1);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

//        Motors Will never stop
        while(distanceSensor1.getDistance(DistanceUnit.INCH)<5.0){
            leftMotor.setPower(0.5);
            rightMotor.setPower(-0.5);
        }





    }

    public static void encoderDrive(){

    }
}
