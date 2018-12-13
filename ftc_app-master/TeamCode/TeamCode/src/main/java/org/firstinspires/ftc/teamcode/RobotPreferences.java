package org.firstinspires.ftc.teamcode;

public class RobotPreferences {

    public static final String motor1 = "motor1";
    public static final String motor2 = "motor2";
    public static final String motor3 = "motor3";
    public static final String motor4 = "motor4";
    public static final String motor5 = "motor5"
    public static final String distanceSensor1 = "distanceSensor1";
    public static final String servo1 = "servo1";
    public static final String servo2 = "servo2";
    public static final Integer encoderCount = 2240;
    public static final double gearReduction = (25 / 15);
    public static final double wheelSize = 3.543;
    public static final double wheelCircumference = wheelSize * Math.PI;
    public static final double robotDiameter = 11.625;
    public static final double robotCircumference = robotDiameter*Math.PI;
    public static final double distancePerPulse = wheelCircumference / (encoderCount * gearReduction);
    public static final double pulsePerInch = 90;

}