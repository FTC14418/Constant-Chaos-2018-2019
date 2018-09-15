package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOpTest", group="Iterative Opmode")
public class TeleopTest extends OpMode{
    RobotPreferences prefs;
    HardwareMap hardwareMap;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;




    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, prefs.motor1);
        rightMotor = hardwareMap.get(DcMotor.class, prefs.motor2);
    }

    @Override
    public void loop(){
         arcadeDrive(-gamepad1.left_stick_y,gamepad1.right_stick_y);

    }

    public void arcadeDrive(double drive , double turn){
        double leftPower = Range.clip(drive + turn, -1.0, 1.0 );
        double rightPower = Range.clip(drive-turn,-1.0, 1.0);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }


}