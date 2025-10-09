package org.firstinspires.ftc.teamcode.ActionTrain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Feeder {
    private CRServo feeder = null;
    private ElapsedTime feederTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 0.80;
    private double feedOn                = 1.0;
    private  double feedOff              = 0.0;

    public Feeder(HardwareMap hardwareMap, String deviceName){
        this.feeder = hardwareMap.get(CRServo.class, deviceName);
    }
    public void initialize(boolean reverse){
        feeder.setPower(feedOff);
        if(reverse){
            feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    public void setRotation(double speed){
        feedOn = speed;
    }
    public void feed_on(){
        feeder.setPower(feedOn);
    }
    public void feed_off(){
        feeder.setPower(feedOff);
    }

}
