package org.firstinspires.ftc.teamcode.RoadRunnerTest;

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
public final class Test1 extends LinearOpMode {
    private static final boolean USE_CAMERA = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private final double DESIRED_DISTANCE = 10;
    double tagX;
    double tagY;
    double tagYaw;
    double tagPitch;
    double tagRange;
    double tagBearing;
    double tagZ;
    boolean targetFound = false;
    double aprilTagSize = 2 * 0.0254; //multiply inches by 0.0254 in terns of meters
    final boolean USE_WEBCAM = true;
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        initAprilTag();
        setManualExposure(3, 100);
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while(!gamepad1.a && opModeIsActive()){
                checkForAprilTag();
                telemetry.update();
            }

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToLinearHeading(new Pose2d(11, 20, Math.toRadians(135)), Math.toRadians(135))
                            .strafeToConstantHeading(new Vector2d(50,-10))
                            .build());

        }
    }
    private void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(965.463,965.463, 310.036, 291.828)
                .build();
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        aprilTag.setDecimation(3);
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void checkForAprilTag(){
        targetFound = false;
        desiredTag = null;
        /*List<AprilTagDetection>currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection: currentDetections){
            if(detection.metadata != null){
                if((DESIRED_TAG_ID>0) || (detection.id == DESIRED_TAG_ID)){
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }else{
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    telemetry.update();
                }
            }else{
                telemetry.addData("Unknown", "Tag ID %d is not in tagLibrary");
                telemetry.update();
            }
        }

         */
        if(aprilTag.getDetections().size()>0){
            desiredTag = aprilTag.getDetections().get(0);
            targetFound = true;
        }
        if(targetFound){
            if(desiredTag != null && desiredTag.rawPose != null) {
                if(desiredTag.ftcPose != null){
                    tagX = desiredTag.ftcPose.x;
                    tagY = desiredTag.ftcPose.y;
                    tagRange = (double) ((double) (desiredTag.ftcPose.range) - DESIRED_DISTANCE);
                    tagYaw = desiredTag.ftcPose.yaw;
                    tagPitch = desiredTag.ftcPose.pitch;
                    tagBearing = desiredTag.ftcPose.bearing;
                    telemetry.addData("x", desiredTag.ftcPose.x);
                    telemetry.addData("y", desiredTag.ftcPose.y);
                    telemetry.addData("yaw", desiredTag.ftcPose.yaw);
                    telemetry.addData("pitch", desiredTag.ftcPose.pitch);
                    telemetry.addData("bearing", desiredTag.ftcPose.bearing);
                    telemetry.addData("range", desiredTag.ftcPose.range);
                }else {
                    tagX = desiredTag.rawPose.x;
                    tagY = desiredTag.rawPose.y;
                    tagZ = desiredTag.rawPose.z;
                    telemetry.addData("x", desiredTag.rawPose.x);
                    telemetry.addData("y", desiredTag.rawPose.y);
                    telemetry.addData("z", desiredTag.rawPose.z);
                }
            }else{
                telemetry.addLine("null");
            }
        }


    }
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

