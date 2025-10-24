package org.firstinspires.ftc.teamcode.ActionTrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public final class FunctionalityTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose                = new Pose2d(0, 0, 0);
        MecanumDrive drive              = new MecanumDrive(hardwareMap, beginPose);
        CameraSystem camera             = new CameraSystem(hardwareMap);
        Firecracker rightFirecracker    =  new Firecracker(hardwareMap, "right_firecracker");
        Firecracker leftFirecracker     =  new Firecracker(hardwareMap, "left_firecracker");
        Inhaler inhaler                 = new Inhaler(hardwareMap, "inhaler");
        Feeder leftFeeder               = new Feeder(hardwareMap, "leftFeeder");
        Feeder rightFeeder              = new Feeder(hardwareMap, "rightFeeder");
        LED leftLight                   = new LED(hardwareMap, "left_light");
        LED middleLight                 = new LED(hardwareMap, "middle_light");
        LED rightLight                  = new LED(hardwareMap, "right_light");
        ColorSensorCode leftColor       = new ColorSensorCode(hardwareMap, "left_color_sensor");
        ColorSensorCode rightColor      = new ColorSensorCode(hardwareMap, "right_color_sensor");
        DriveMotorTest test1            = new DriveMotorTest(hardwareMap);
        boolean reverse = true;
        boolean notReverse = false;
        int detectedID;

        //adjust reverse if needed
        leftFeeder.initialize(reverse);
        rightFeeder.initialize(notReverse);
        leftFirecracker.initialize(reverse);
        rightFirecracker.initialize(notReverse);
        inhaler.initialize(notReverse);
        test1.initialize(true);
        camera.cameraOn();

        waitForStart();

        while(opModeIsActive()){
            //id 21 GPP
            //id 23 PPG
            //id 22 PGP
            if(gamepad1.a) {
                test1.motorTest(0.5);
            }
            if(gamepad1.b){
                test1.stopMotor();
            }
            if(gamepad1.x){

            }
            if(gamepad1.right_bumper){
                detectedID = camera.getPattern();
                if(detectedID == 21){
                    leftLight.setGreen();
                    middleLight.setPurple();
                    rightLight.setPurple();
                }
                if(detectedID == 22){
                    leftLight.setPurple();
                    middleLight.setGreen();
                    rightLight.setPurple();
                }
                if(detectedID == 23) {
                    leftLight.setPurple();
                    middleLight.setPurple();
                    rightLight.setGreen();
                }
            }
            if(gamepad1.left_bumper){
                leftLight.lightOff();
                middleLight.lightOff();
                rightLight.lightOff();
            }
        }
        /*if (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToLinearHeading(new Pose2d(11, 20, Math.toRadians(135)), Math.toRadians(135))
                            .strafeToConstantHeading(new Vector2d(50,-10))
                            .build());

        }*/
    }

}

