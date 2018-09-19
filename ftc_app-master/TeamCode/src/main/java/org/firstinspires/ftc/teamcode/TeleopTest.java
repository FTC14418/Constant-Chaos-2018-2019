package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpTest", group="Iterative Opmode")
public class TeleopTest extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor1);
        rightMotor = hardwareMap.get(DcMotor.class, RobotPreferences.motor2);
    }

    @Override
    public void loop(){
         arcadeDrive(-gamepad1.right_stick_x, gamepad1.left_stick_y);

    }

    public void arcadeDrive(double drive , double turn){
        double leftPower = Range.clip(drive + turn, -1.0, 1.0 );
        double rightPower = Range.clip(drive-turn,-1.0, 1.0);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }
}