package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ArmCommand extends CommandBase {

    double target;

    public ArmCommand(double target) {
        this.target = target;
    }

    @Override
    public void initialize() {
        RobotHardware.getInstance().extension.setArmTargetPosition(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
