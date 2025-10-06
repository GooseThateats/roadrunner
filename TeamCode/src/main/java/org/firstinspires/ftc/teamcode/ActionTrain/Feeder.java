package org.firstinspires.ftc.teamcode.ActionTrain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Feeder {
    private CRServo feeder = null;

    public static final double FEED_ON                = 0.0;
    public static final double FEED_OFF               = 0.0;

    public Feeder(HardwareMap hardwareMap, String deviceName){
        this.feeder = hardwareMap.get(CRServo.class, deviceName);
    }
    public void feed_on(){
        feeder.setPower(FEED_ON);
    }
    public void feed_off(){
        feeder.setPower(FEED_OFF);
    }

}
