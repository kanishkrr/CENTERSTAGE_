package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo angleServo, rightServo, leftServo;
    private static final double servoOpen = 0.3;

    public Claw(HardwareMap hMap) {
        angleServo = hMap.get(Servo.class, "servo5");
        rightServo = hMap.get(Servo.class, "servo2");
        leftServo = hMap.get(Servo.class, "servo3");
    }

    public void openRightNarrow() {

    }

    public void openLeftNarrow() {

    }

    public void openRightWide() {

    }

    public void openLeftWide() {

    }

    public void alignWithBoard() {

    }

    public void alignWithGround() {

    }

    public void closeRight() {

    }

    public void closeLeft() {

    }

    public void resetPosition() {

    }

    public void closeBoth() {
        closeLeft();
        closeRight();
    }

    public void openBothNarrow() {
        openLeftNarrow();
        openRightNarrow();
    }

    public void openBothWide() {
        openLeftWide();
        openRightWide();
    }


}
