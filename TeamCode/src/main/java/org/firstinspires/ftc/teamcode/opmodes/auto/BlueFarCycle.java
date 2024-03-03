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
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontExtendCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhiteFrontRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.cyclecommand.WhitePixelPlaceCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurpleDropRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.PurplePixelDropCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.closesidecommand.YellowPixelRetractCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.farsidecommand.YellowPixelFarCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand.InitCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.autocommand.generalcommand.StartCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.drivecommand.PositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.drive.Constants;
import org.firstinspires.ftc.teamcode.common.drive.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.centerstage.Location;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.OpModeVisionBlueFar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Config
@Autonomous(name = "blue far cycle")
public class BlueFarCycle extends LinearOpMode {

    OpModeVisionBlueFar pipeline;
    OpenCvWebcam camera;

    private final RobotHardware robot = RobotHardware.getInstance();


    private List<LynxModule> allHubs;

    @Override
    public void runOpMode() {

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewID);
        pipeline = new OpModeVisionBlueFar();
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
        Pose whitePose = new Pose();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Globals.IS_AUTO = true;
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;
        Globals.PATH = Location.SIDE;
        Globals.PARK = Location.RIGHT;

        Constants.MAX_LINEAR_SPEED = 0.5;
        Constants.MAX_ROTATIONAL_SPEED = 0.4;

        robot.init(hardwareMap);

        robot.claw.setClawState(IntakeSubsystem.Mode.CLOSE, IntakeSubsystem.Mode.BOTH);
        robot.claw.changeAngleState(IntakeSubsystem.Mode.REST);

        robot.claw.update(0);

        CommandScheduler.getInstance().schedule(new InitCommand());


        while (opModeInInit()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            OpModeVisionBlueFar.TeamPropLocation initobjLocation = pipeline.location;

            if (initobjLocation == OpModeVisionBlueFar.TeamPropLocation.Middle) {
                telemetry.addData("block detected in center", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionBlueFar.TeamPropLocation.Right) {
                telemetry.addData("block detected in right", initobjLocation);
                telemetry.update();
            } else if (initobjLocation == OpModeVisionBlueFar.TeamPropLocation.Left) {
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

        OpModeVisionBlueFar.TeamPropLocation location = pipeline.location;
        int numberOfLeft = 0;
        int numberOfRight = 0;
        int numberOfCenter = 0;
        for (int i=0; i<100; i++) {
            location = pipeline.location;
            if (location == OpModeVisionBlueFar.TeamPropLocation.Middle) {
                numberOfCenter += 1;
                telemetry.addData("block detected in center", location);
                telemetry.update();
            }
            else if (location == OpModeVisionBlueFar.TeamPropLocation.Right) {
                numberOfRight += 1;
                telemetry.addData("block detected in right", location);
                telemetry.update();
            }
            else if (location == OpModeVisionBlueFar.TeamPropLocation.Left) {
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
                purplePose = new Pose(0.5, 3, Math.toRadians(31));
                yellowPose = new Pose(-78, 12, Math.toRadians(97)); //yellowPose = new Pose(-78, 12, Math.toRadians(97));
                whitePose = new Pose(-81, 20, Math.toRadians(100));
                break;
            case Middle:
                purplePose = new Pose(1, 9.2, Math.toRadians(16));
                yellowPose = new Pose(-78, 20, Math.toRadians(100));
                whitePose = new Pose(-80, 10, Math.toRadians(100));

                break;
            case Right:
                purplePose = new Pose(1.5, 10, Math.toRadians(-12.5));
                yellowPose = new Pose(-81, 29, Math.toRadians(100));
                whitePose = new Pose(-78, 13.5, Math.toRadians(100));
                break;
        }


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new StartCommand(),

                        new PositionCommand(purplePose)
                                .alongWith(new PurplePixelDropCommand()),

                        new ClawCommand(IntakeSubsystem.Mode.WIDE, IntakeSubsystem.Mode.RIGHT),

                        new WaitCommand(200),

                        new PurpleDropRetractCommand(),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new PositionCommand(new Pose(purplePose.x, purplePose.y, Math.toRadians(-90))),
                                        new PositionCommand(new Pose(5.5, 29.1, Math.toRadians(-92.5)))
                                ),
                                new SequentialCommandGroup(),
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),

                        //new PositionCommand(new Pose(purplePose.x, purplePose.y, Math.toRadians(0))), //for now

                        new PositionCommand(new Pose(5.5, 29.1, Math.toRadians(-92.5))),

                        new WhiteFrontExtendCommand(),

                        new WhiteFrontRetractCommand(),

                        new WaitCommand(4000),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new PositionCommand(new Pose(3, 1, Math.toRadians(-89))),
                                        new PositionCommand(new Pose(-60, -1, Math.toRadians(-87)))
                                ),
                                new SequentialCommandGroup(
                                        new PositionCommand(new Pose(-3, 48, Math.toRadians(-90))),
                                        new PositionCommand(new Pose(-60, 48, Math.toRadians(-90)))
                                ),
                                () -> {
                                    return Globals.PATH == Location.SIDE;
                                }
                        ),


                        new PositionCommand(yellowPose),

                        new YellowPixelFarCommand(),

                        new WaitCommand(50),

                        new YellowPixelRetractCommand(),

                        new PositionCommand(whitePose),

                        new WhitePixelPlaceCommand(),

                        new YellowPixelRetractCommand(),

                        new WaitCommand(500),

                        new ConditionalCommand(
                                new PositionCommand(new Pose(-80, 0, Math.toRadians(100))),
                                new PositionCommand(new Pose(-88, 44, Math.toRadians(100))),
                                () -> {
                                    return Globals.PARK == Location.LEFT;
                                }
                        )
                )
        );


        while (!isStopRequested() && opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            robot.update();
            CommandScheduler.getInstance().run();

            telemetry.addData("arm current", robot.extension.armCurrent);
            telemetry.update();
        }

        robot.kill();

    }
}
