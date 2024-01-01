package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    DcMotor arm;
    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.dcMotor.get("Arm_Motor");
    }

    public void setHeight(int target, double power) {
        arm.setTargetPosition(target);

        int error = arm.getTargetPosition()-arm.getCurrentPosition();

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (error < 0) {
            arm.setPower(-power);
        } else {
            arm.setPower(power);
        }

        while (arm.isBusy()) {
            System.out.println(arm.getCurrentPosition());
        }

        arm.setPower(0.0);
    }

    public int getPos() {
        return arm.getCurrentPosition();
    }

    public int getTargetPos() {
        return arm.getTargetPosition();
    }
}
