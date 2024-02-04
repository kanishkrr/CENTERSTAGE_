package org.firstinspires.ftc.teamcode.common.centerstage;

import org.firstinspires.ftc.teamcode.common.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.common.subsystems.Claw;
import org.firstinspires.ftc.teamcode.common.subsystems.ExtensionSystem;

public class CommandBase {

    ExtensionSystem extensionSys;
    Claw claw;
    Actuator actuator;

    public CommandBase(ExtensionSystem extensionSystem, Claw claw, Actuator actuator) {
        this.extensionSys = extensionSystem;
        this.claw = claw;
        this.actuator = actuator;
    }

    public void pickUpPixelCommand() {
        extensionSys.setArmTarget(30);
        extensionSys.setSlideTarget(400);
        claw.openBothWide();
    }

    public void autoExtendCommand() {
        extensionSys.setArmTarget(100);
        extensionSys.setSlideTarget(600);
    }

    public void hangCommand() {

    }

    public void hangRelease() {

    }
}
