package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class OneTapPush extends OpMode {

    Servo Pusher;

    public void push(){
        boolean g1rb = gamepad1.right_bumper;
        boolean toggleReady = true;
        double servoArmPos = Pusher.getPosition();

        boolean mov1 = false;
        boolean mov2 = false;

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
//                    telemetry.addData("Counter is", "push " + i);

                }
            }
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }



    @Override
    public void init() {
        Pusher = hardwareMap.get(Servo.class, "Pusher");
    }


    @Override
    public void init_loop() {

        Pusher.setPosition(0.0);
    }


    @Override
    public void loop() {
        push();
    }

}
