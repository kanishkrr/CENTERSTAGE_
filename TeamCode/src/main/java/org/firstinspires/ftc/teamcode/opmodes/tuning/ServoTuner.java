package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "ServoTuner")
public class ServoTuner extends OpMode {


    Servo rightServo, leftServo, angleServo;
    public static double rightServoTarget = 0;
    public static double leftServoTarget = 0;
    public static double angleServoTarget = 0;
    public static int symmetrical = 1;


    @Override
    public void init() {
        leftServo = hardwareMap.get(Servo.class, "ls");
        rightServo = hardwareMap.get(Servo.class, "rs");
        angleServo = hardwareMap.get(Servo.class, "as");
    }

    public void loop() {
        telemetry.addData("Status", "Run Time: " + getRuntime());
        telemetry.update();

        double rsPos;
        double lsPos;
        double asPos;

        if (symmetrical == 1) {
            lsPos = 1-rightServoTarget;
        } else {
            lsPos = leftServoTarget;
        }

        rsPos = rightServoTarget;
        asPos = angleServoTarget;

        rightServo.setPosition(rsPos);
        leftServo.setPosition(lsPos);
        angleServo.setPosition(asPos);
    }
}