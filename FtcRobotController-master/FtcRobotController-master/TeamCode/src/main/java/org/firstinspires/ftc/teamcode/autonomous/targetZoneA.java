package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class targetZoneA extends OpMode {




        BNO055IMU imu;
        Orientation angles;
        private DcMotor RFMotor;
        private DcMotor LFMotor;
        private DcMotor LBMotor;
        private DcMotor RBMotor;

        ElapsedTime t1 = ElapsedTime()


    @Override
    public void init() {

    }

    @Override
    public void init_loop(){

    }


    @Override
    public void start() {

    }

    @Override
    public void loop(){

    }
}
