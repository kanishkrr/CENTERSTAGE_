package org.firstinspires.ftc.teamcode.common.centerstage;

import org.firstinspires.ftc.teamcode.common.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.common.subsystems.Claw;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSystem;

public class CommandBase {

    ExtensionSystem extension;
    Claw claw;
    Actuator actuator;

    public CommandBase(ExtensionSystem extensionSystem, Claw claw, Actuator actuator) {
        this.extension = extensionSystem;
        this.claw = claw;
        this.actuator = actuator;
    }

    //resets everything at the start
    public void periodic() {

    }

    public void pickUpPixelCommand() {
        extension.setArmTarget(30);
        extension.setSlideTarget(400);
        claw.openBothWide();
    }

    public void autoExtendCommand() {
        extension.setArmTarget(100);
        extension.setSlideTarget(600);
    }

    public void hangCommand() {

    }

    public void hangRelease() {

    }

    public void leftJoystickCommand(double leftStickY) {

    }

    public void rightJoystickCommand(double rightStickY) {

    }


}
