package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangSubsystem {

    DcMotor actuator;
    public HangSubsystem(HardwareMap hardwareMap) {
        actuator = hardwareMap.dcMotor.get("Actuator_Motor");

        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        actuator.setPower(power);
    }
}
