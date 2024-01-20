package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "ServoTuner")
public class ServoTuner extends OpMode {


    Servo angleServoOne, angleServoTwo;
    public static double targetPos = 0;
    double angleServoOneTargetPos = 0;
    double angleServoTwoTargetPos = 0;


    @Override
    public void init() {
        angleServoOne = hardwareMap.get(Servo.class, "servo1");
        angleServoTwo = hardwareMap.get(Servo.class, "servo5");
    }

    public void loop() {
        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        angleServoOneTargetPos = targetPos;

        angleServoOne.setPosition(angleServoOneTargetPos);
        angleServoTwoTargetPos = 1-angleServoOneTargetPos;
        angleServoTwo.setPosition(angleServoTwoTargetPos);
    }
}