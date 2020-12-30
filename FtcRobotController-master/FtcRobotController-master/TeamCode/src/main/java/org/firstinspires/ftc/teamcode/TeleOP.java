 package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOP extends OpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Shooter;
    private Servo Pusher;

    ElapsedTime t1 = new ElapsedTime();

    boolean flyWeel = false;

   public void moveDriveTrain(){
       if(gamepad1.left_bumper){
           LFMotor.setPower(0.6*(gamepad1.right_stick_y));
           LBMotor.setPower(0.6*(gamepad1.right_stick_y));
           RFMotor.setPower(0.6*(-gamepad1.left_stick_y));
           RBMotor.setPower(0.6*(-gamepad1.left_stick_y));
       }else {
           LFMotor.setPower(gamepad1.right_stick_y);
           LBMotor.setPower(gamepad1.right_stick_y);
           RFMotor.setPower(-gamepad1.left_stick_y);
           RBMotor.setPower(-gamepad1.left_stick_y);
       }
   }

   public void shoot() {
       if (gamepad1.right_trigger > 0.5 && t1.seconds() > 0.5)
           if (!flyWeel) {
               Shooter.setPower(-1);
                flyWeel = true;
           } else if (flyWeel){
                Shooter.setPower(0);
                flyWeel = false;
           }
   }

    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void push(){
        boolean g1rb = gamepad1.right_bumper;
        boolean toggleReady = true;
        double servoArmPos = Pusher.getPosition();

        if(g1rb == false){
            toggleReady = true;
        }
        if(g1rb && toggleReady){
            toggleReady = false;
            if (servoArmPos == 0.0 && gamepad1.right_bumper){
                for (int i=0; i<3; i++){
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.27);
                    sleep(330);
                    Pusher.setPosition(0.0);
                    sleep(400);

                }
            }
        }
    }

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Pusher = hardwareMap.get(Servo.class, "Pusher");

    }

    @Override
    public void init_loop() {

        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
    }

    @Override
    public void loop() {
        moveDriveTrain();
        push();
        shoot();

    }

}
