package org.firstinspires.ftc.teamcode.autonomous;//package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.ScanRings;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class DeliverWobble extends OpMode {

    ScanRings sense = new ScanRings();
    autoMovements run = new autoMovements();

    double one = 537.6;

    public void Move (){
        if (sense.moveSingle()) {
            sense.rampUp(one,0, 0.5, 0.3);
            telemetry.addData("Scanned: ", "one");
        } else if (sense.moveQuad()) {
            sense.rampUp(2*one, 0, 0.5, 0.3);
            telemetry.addData("Scanned: ", "four");
        } else if (sense.moveNone()) {
            sense.rampUp(3*one, 0, 0.5, 0.3);
            telemetry.addData("Scanned: ", "none");
        }
    }

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
        sense.scan();
        Move();

telemetry.update();

    }
}
