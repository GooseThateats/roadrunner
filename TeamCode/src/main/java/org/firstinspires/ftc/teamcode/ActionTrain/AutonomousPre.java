package org.firstinspires.ftc.teamcode.ActionTrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public final class AutonomousPre extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose                = new Pose2d(0, 0, 0);
        MecanumDrive drive              = new MecanumDrive(hardwareMap, beginPose);
        CameraSystem camera             = new CameraSystem(hardwareMap);
        Firecracker rightFirecracker    =  new Firecracker(hardwareMap, "rightFirecracker");
        Firecracker leftFirecracker     =  new Firecracker(hardwareMap, "leftFirecracker");
        Inhaler inhaler                 = new Inhaler(hardwareMap, "inhaler");
        Feeder leftFeeder               = new Feeder(hardwareMap, "leftFeeder");
        Feeder rightFeeder               = new Feeder(hardwareMap, "rightFeeder");

        waitForStart();

        if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToLinearHeading(new Pose2d(11, 20, Math.toRadians(135)), Math.toRadians(135))
                            .strafeToConstantHeading(new Vector2d(50,-10))
                            .build());

        }
    }

}

