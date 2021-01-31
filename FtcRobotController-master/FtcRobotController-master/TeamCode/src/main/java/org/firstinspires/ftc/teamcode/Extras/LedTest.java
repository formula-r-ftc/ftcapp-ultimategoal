package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LedTest extends OpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private RevBlinkinLedDriver ColorLights;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();
    ElapsedTime t3 = new ElapsedTime();
    ElapsedTime t4 = new ElapsedTime();

    public void moveDriveTrain(){
        if(gamepad1.left_bumper){
            LFMotor.setPower(0.3*(gamepad1.right_stick_y));
            LBMotor.setPower(0.3*(gamepad1.right_stick_y));
            RFMotor.setPower(0.3*(-gamepad1.left_stick_y));
            RBMotor.setPower(0.3*(-gamepad1.left_stick_y));
        }else {
            LFMotor.setPower(gamepad1.right_stick_y);
            LBMotor.setPower(gamepad1.right_stick_y);
            RFMotor.setPower(-gamepad1.left_stick_y);
            RBMotor.setPower(-gamepad1.left_stick_y);
        }
    }

    boolean rotation () {
        double AVGPower = (RFMotor.getPower() + RBMotor.getPower() + LFMotor.getPower() + LBMotor.getPower())/4;
        if (AVGPower == 0) {
            return false;
        } else {
            return true;
        }

    }

    public void colorLights() {

        if (rotation() == false) {
            ColorLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (rotation() == true) {
            ColorLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }
    }

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start(){
        t2.reset();

    }

    @Override
    public void loop() {
        moveDriveTrain();

    }
}
