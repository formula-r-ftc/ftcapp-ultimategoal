package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOP extends OpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;

   public void moveDriveTrain(){
       if(gamepad1.left_bumper){
           RFMotor.setPower(0.3*(gamepad1.right_stick_y));
           RBMotor.setPower(0.3*(gamepad1.right_stick_y));
           LFMotor.setPower(0.3*(-gamepad1.left_stick_y));
           LBMotor.setPower(0.3*(-gamepad1.left_stick_y));
       }else {
           RFMotor.setPower(gamepad1.right_stick_y);
           RBMotor.setPower(gamepad1.right_stick_y);
           LFMotor.setPower(-gamepad1.left_stick_y);
           LBMotor.setPower(-gamepad1.left_stick_y);
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
    public void loop() {
        moveDriveTrain();

    }

}
