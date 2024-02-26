package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PathCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

import java.util.List;

@Config
@Autonomous(name = "path route test")
public class PATHTEST extends LinearOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();

    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {


        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        CommandScheduler.getInstance().schedule(new ArmCommand(160));


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            CommandScheduler.getInstance().run();
            robot.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        //new Pose(24, 30, Math.toRadians(-90))

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new PathCommand(new Pose(0, 20, 0), new Pose(-20, 40, 0))
                )
        );


        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            robot.update();
            CommandScheduler.getInstance().run();
        }

        robot.kill();

    }
}
