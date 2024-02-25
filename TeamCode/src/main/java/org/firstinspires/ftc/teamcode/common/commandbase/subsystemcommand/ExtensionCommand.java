package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class ExtensionCommand extends CommandBase {

    double target;

    public ExtensionCommand(double target) {
        this.target = target;
    }

    @Override
    public void initialize() {
        RobotHardware.getInstance().extension.setSlideTargetPosition(target);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
