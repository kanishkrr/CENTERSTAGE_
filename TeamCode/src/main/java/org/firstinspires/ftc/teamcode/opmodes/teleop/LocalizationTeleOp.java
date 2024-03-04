package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name = "Localization")
public class LocalizationTeleOp extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;

        robot.init(hardwareMap);

        robot.extension.setArmTargetPosition(180);
        robot.extension.setSlideTargetPosition(0);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.update();

        robot.drivetrain.set(new Pose(gamepad1.left_stick_x*(gamepad1.right_trigger*0.75 + 0.25), -gamepad1.left_stick_y*(gamepad1.right_trigger*0.75 + 0.25), gamepad1.right_stick_x*(gamepad1.right_trigger*0.5 + 0.4)), 0);

        robot.extension.setArmTargetPosition(robot.extension.armTarget - gamepad2.left_stick_y*15);
        robot.extension.setSlideTargetPosition(robot.extension.slideTarget + gamepad2.right_stick_y*30);

        telemetry.addData("robot x:", robot.drivetrain.localizer.getPose().x);
        telemetry.addData("robot y:", robot.drivetrain.localizer.getPose().y);
        telemetry.addData("robot heading:", robot.drivetrain.localizer.getPose().heading);
        telemetry.addData("arm height:", robot.extension.armCurrent);
        telemetry.addData("extension height:", robot.extension.slideCurrent);
        telemetry.update();

    }
}
