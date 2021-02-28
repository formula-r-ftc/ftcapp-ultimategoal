package org.firstinspires.ftc.teamcode.Extras;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

@Autonomous
public class testSlides extends OpMode {

    private DcMotor slides;

    ElapsedTime t1 = new ElapsedTime();

    double linearSlideInitPos = 0;

    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double distance = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
        telemetry.addData("LS distance", distance);
        double power = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
        return power;
    }

    public void slidePos(double position, double time, double maxSpeed){
        double Acceleration = maxSpeed/time;
        double power = t1.seconds() * Acceleration;
        if (Math.abs(power) < linearSlideEncSpeed(position, maxSpeed)){
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        } else {
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        }
    }

    @Override
    public void init() {

        slides = hardwareMap.get(DcMotor.class, "slides");

        linearSlideInitPos = slides.getCurrentPosition();
    }

    @Override
    public void init_loop() {

        telemetry.addData("slides Pos: ", slides.getCurrentPosition());
    }



    @Override
    public void loop() {
        slidePos(800, 0.5,0.75);
        telemetry.addData("slides pos: ", slides.getCurrentPosition());
        telemetry.update();
    }
}
