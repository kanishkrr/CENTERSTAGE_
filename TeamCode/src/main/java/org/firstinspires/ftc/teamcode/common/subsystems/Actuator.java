package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actuator {

    DcMotor actuator;
    public Actuator(HardwareMap hardwareMap) {
        actuator = hardwareMap.dcMotor.get("Actuator_Motor");
    }

    public void setPower(double power) {
        actuator.setPower(power);
    }
}
