package org.firstinspires.ftc.teamcode.ActionTrain;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hammer {
    private Servo hammer = null;
    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;

    public Hammer(HardwareMap hardwareMap, String deviceName){
        this.hammer = hardwareMap.get(Servo.class, deviceName);
    }

    public void initialize(){
        hammer.setPosition(LEFT_POSITION);
    }
    public void setRight(){
        hammer.setPosition(RIGHT_POSITION);
    }

    public void setLeft(){
        hammer.setPosition(LEFT_POSITION);
    }

}
