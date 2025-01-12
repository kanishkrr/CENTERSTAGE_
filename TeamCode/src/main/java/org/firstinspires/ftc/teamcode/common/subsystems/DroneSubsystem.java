package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubsystem {

    Servo servo;

    public DroneSubsystem(HardwareMap hMap) {
        servo = hMap.get(Servo.class, "ps");
    }

    public void init() {
        servo.setPosition(0);
    }

    public void release() {
        servo.setPosition(0.4);
    }


}
