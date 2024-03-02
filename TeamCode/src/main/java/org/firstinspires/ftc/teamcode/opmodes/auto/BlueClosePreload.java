package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.BlueParkCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.PurplePixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand.InitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueClose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


@Config
@Autonomous(name = "blue close preload")
public class BlueClosePreload extends LinearOpMode {

    OpModeVisionBlueClose pipeline;
    OpenCvWebcam camera;

    private final RobotHardware robot = RobotHardware.getInstance();


    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewID);
        pipeline = new OpModeVisionBlueClose();
        camera.setPipeline(pipeline);

        camera.setMillisecondsPermissionTimeout(Integer.MAX_VALUE); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("No Camera Could Not Start");
                runOpMode();
            }
        });
        telemetry.addLine("Waiting for start");
        telemetry.update();


        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);


        Pose purplePose = new Pose();
        Pose yellowPose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.CLOSE;

        /*
        ramp up max speed when relocalize command works + perfect lateral distance is calculated
         */

        Constants.MAX_LINEAR_SPEED = 0.7;
        Constants.MAX_ROTATIONAL_SPEED = 0.5;

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);


        CommandScheduler.getInstance().schedule(new InitCommand());


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            OpModeVisionBlueClose.TeamPropLocation initobjLocation = pipeline.location;

            if (initobjLocation == OpModeVisionBlueClose.TeamPropLocation.Middle) {
                telemetry.addData("block detected in center", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionBlueClose.TeamPropLocation.Right) {
                telemetry.addData("block detected in right", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionBlueClose.TeamPropLocation.Left) {
                telemetry.addData("block detected in left", initobjLocation);
                telemetry.update();
            } else {
                telemetry.addData("no block detected", initobjLocation);
                telemetry.update();
            }

            CommandScheduler.getInstance().run();
            robot.update();

        }

        waitForStart();
        if (isStopRequested()) return;

        OpModeVisionBlueClose.TeamPropLocation location = pipeline.location;
        int numberOfLeft = 0;
        int numberOfRight = 0;
        int numberOfCenter = 0;
        for (int i=0; i<100; i++) {
            location = pipeline.location;
            if (location == OpModeVisionBlueClose.TeamPropLocation.Middle) {
                numberOfCenter += 1;
                telemetry.addData("block detected in center", location);
                telemetry.update();
            }
            else if (location == OpModeVisionBlueClose.TeamPropLocation.Right) {
                numberOfRight += 1;
                telemetry.addData("block detected in right", location);
                telemetry.update();
            }
            else if (location == OpModeVisionBlueClose.TeamPropLocation.Left) {
                numberOfLeft += 1;
                telemetry.addData("block detected in left", location);
                telemetry.update();
            }
            else {
                telemetry.addData("no block detected", location);
                telemetry.update();
            }}
        if (numberOfLeft>numberOfCenter && numberOfLeft>numberOfRight) {
            //insert code for left auto here
            telemetry.addData("following left trajectory", location);
            telemetry.update();
        }
        else if (numberOfRight>numberOfLeft && numberOfRight>numberOfCenter) {
            //insert code for right auto here
            telemetry.addData("following right trajectory", location);
            telemetry.update();
        }
        else if (numberOfCenter>numberOfLeft && numberOfCenter>numberOfRight) {
            //insert code for center auto here
            telemetry.addData("following center trajectory", location);
            telemetry.update();
        }
        else {
            telemetry.addData("no trajectory being followed", location);
            telemetry.update();
        }

        switch(location) {
            case Left:
                purplePose = new Pose(-27, 46.2, Math.toRadians(86));
                yellowPose = new Pose(-25.3, 22.5, Math.toRadians(86));
                break;
            case Middle:
                purplePose = new Pose(-21, 45.5, Math.toRadians(86));
                yellowPose = new Pose(-24.8, 28, Math.toRadians(86));
                break;
            case Right:
                purplePose = new Pose(-13, 33, Math.toRadians(86));
                yellowPose = new Pose(-24.8, 33.6, Math.toRadians(86));
                break;
        }

        final OpModeVisionBlueClose.TeamPropLocation l = location;


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new ArmCommand(220),

                        new PositionCommand(yellowPose)
                                .alongWith(new YellowPixelExtendCommand()),

                        new WaitCommand(500),

                        new YellowPixelRetractCommand(),

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelExtendCommand()),

                        new ConditionalCommand(
                                new PositionCommand(new Pose(purplePose.x+7.5, purplePose.y, purplePose.heading)),
                                new WaitCommand(50),
                                () -> {
                                    return l == OpModeVisionBlueClose.TeamPropLocation.Right;
                                }
                        ),

                        new ClawCommand(IntakeSubsystem.Mode.LEFT, IntakeSubsystem.Mode.SHARP),

                        new WaitCommand(500),

                        new BlueParkCommand()
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
