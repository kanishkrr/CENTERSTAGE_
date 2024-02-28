package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.DroneSubsystem;

public class DroneCommand extends CommandBase {

    DroneSubsystem drone;
    private final RobotHardware robot = RobotHardware.getInstance();
    public DroneCommand() {
        this.drone = robot.drone;
    }

    @Override
    public void initialize() {
        drone.release();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
