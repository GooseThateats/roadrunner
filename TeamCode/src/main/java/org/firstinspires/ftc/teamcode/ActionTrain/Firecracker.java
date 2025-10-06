package org.firstinspires.ftc.teamcode.ActionTrain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Firecracker {
    private DcMotor firecracker = null;
    private Feeder feeder;

    public Firecracker(HardwareMap hardwareMap, String deviceName) {
        // Initialize arm motor
        firecracker = hardwareMap.get(DcMotor.class, deviceName);
        firecracker.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust if necessary
        firecracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firecracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        firecracker.setPower(0);
    }
    public void crackDaFire(double speed){
        firecracker.setPower(speed);
    }
    public void ceaseFire(){
        firecracker.setPower(0);
    }

}
