package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@Autonomous(name="Auto1", group="Color1")

public class Auto1 extends LinearOpMode{
    DcMotor motor1, motor2, motor3,motor4;
    Servo servo2;
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
        servo2 = hardwareMap.get(Servo.class, RobotPreferences.servo2);
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
//        servo2.setPosition(-90);
        encoderDrive(0.25,0.25, 6, 6, 1000);
        encoderDrive(0.25,  -0.25,4.565, -4.565, 1000);
        encoderDrive(0.25, 0.25,36, 36, 3000);
        encoderDrive(-0.25, 0.25,-9.130, 9.130, 1000);
        encoderDrive(0.25, 0.25, 40, 40, 1000);
        servo2.setPosition(90);
        sleep(3000);
        encoderDrive(-1.0,-1.0, -96, -96, 1000);
    }
    public void encoderDrive(double leftSpeed, double rightSpeed, double leftInches, double rightInches, double timeout) {
        encoderStopAndResetEncoder();
        int newLeftTarget1, newLeftTarget2;
        int newRightTarget1, newRightTarget2;
        if (opModeIsActive()) {
            newLeftTarget1 = (int) (leftInches * pulsePerInch);
            newLeftTarget2 = (int) (leftInches * pulsePerInch);
            newRightTarget1 = (int) (rightInches * pulsePerInch);
            newRightTarget2 = (int) (rightInches * pulsePerInch);
            encoderSetTargetPosition(newLeftTarget1, newLeftTarget2, newRightTarget1, newRightTarget2);
            encoderRunToTargetPosition();
            runtime.reset();
            while ((Math.abs(motor2.getCurrentPosition() )< Math.abs(newLeftTarget2)) && (Math.abs(motor4.getCurrentPosition()) < Math.abs(newRightTarget2)) && (Math.abs(motor1.getCurrentPosition()) < Math.abs(newRightTarget1)) && (Math.abs(motor3.getCurrentPosition()) < Math.abs(newLeftTarget1))) {
                encoderSetPower(leftSpeed, rightSpeed);
            }
            encoderSetPower(0,0);
            sleep(1000);

//            while (opModeIsActive() && runtime.seconds() < timeout && motor1.isBusy() && motor3.isBusy()) {
//                telemetry.addData("Running: ", "Running at %7d: %7d", motor1.getCurrentPosition(), motor3.getCurrentPosition());
//                telemetry.update();
//            }
        }
    }

    public void encoderDriveAccelerometer(double speed, double time){
        autotime.reset();
        autotime.startTime();

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
        }

        public void encoderSetPower(double leftSpeed, double rightSpeed){
            motor2.setPower(leftSpeed);
            motor4.setPower(rightSpeed);
            motor1.setPower(leftSpeed);
            motor3.setPower(rightSpeed);
        }

//    public void encoderDriveAccelerometer(double speed, double time){
//        autotime.reset();
//        autotime.startTimeNanoseconds();
//        while(accelUnit > imu.getAcceleration()){
//            if (time == autotime.time()) {
//                accelUnit = imu.getAcceleration();
//            }
//        }
//
//    }
}
