package org.firstinspires.ftc.teamcode.ActionTrain;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Inhaler {
    private CRServo inhale = null;

    public static final double INHALE_ON                = 0.0;
    public static final double INHALE_OFF               = 0.0;
    public static final double OUTWARD                  = 0.0;

    public Inhaler(HardwareMap hardwareMap){
        this.inhale = hardwareMap.get(CRServo.class, "inhale");
    }
    public void inhale_on(){
        inhale.setPower(INHALE_ON);
    }
    public void inhale_off(){
        inhale.setPower(INHALE_OFF);
    }
    public void outtake(){
        inhale.setPower(OUTWARD);
    }
}
