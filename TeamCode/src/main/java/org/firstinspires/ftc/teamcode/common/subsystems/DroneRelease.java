package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneRelease {

    Servo servo;

    public DroneRelease(HardwareMap hMap) {
        servo = hMap.get(Servo.class, "servo5");
    }

    public void init() {
        servo.setPosition(0);
    }

    public void release() {
        servo.setPosition(0.4);
    }


}
