package org.firstinspires.ftc.teamcode.ActionTrain;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Firecracker {
    private DcMotorEx firecracker = null;
    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.


    public Firecracker(HardwareMap hardwareMap, String deviceName) {
        // Initialize arm motor
        firecracker = hardwareMap.get(DcMotorEx.class, deviceName);

    }

    public void initialize(boolean reverse){
        if(reverse){
            firecracker.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust if necessary
        }
        firecracker.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        firecracker.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        firecracker.setZeroPowerBehavior(BRAKE);
        firecracker.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        firecracker.setPower(0);

    }
    public void crackDaFire(){
        firecracker.setPower(LAUNCHER_CLOSE_TARGET_VELOCITY);
    }

    public void crackBigBoyFire(){
        firecracker.setPower(LAUNCHER_FAR_TARGET_VELOCITY);
    }

    public void ceaseFire(){
        firecracker.setPower(0);
    }

    public double getCurrentVelocity(){
        return firecracker.getVelocity();
    }

}
