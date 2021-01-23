package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ForLoop extends OpMode {
    Servo arm;

    private DcMotor RFMotor;
    public void forLoop(){
        for (int i = 100; i>=8; i--){ // 1, 2, 3, 4, 5, 6, 7
            System.out.println(i);
        }
    }

    @Override
    public void init(){

        arm.setPosition(0.0);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop(){

    }
}
