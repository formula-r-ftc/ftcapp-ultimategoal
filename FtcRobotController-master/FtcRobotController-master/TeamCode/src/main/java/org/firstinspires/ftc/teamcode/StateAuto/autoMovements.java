  package org.firstinspires.ftc.teamcode.StateAuto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Base64;
import java.util.List;

  @Autonomous
public class autoMovements extends OpMode {

      private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
      private static final String LABEL_FIRST_ELEMENT = "Quad";
      private static final String LABEL_SECOND_ELEMENT = "Single";

      private static final String VUFORIA_KEY =
              "ASIwwWv/////AAABmR/+9d4sSkVEshzTIOkfUgAWTcQCqWQ3NeZFwrYj+HewIITQOcdzK95pLGiq3w+muSW12YMucPY4gr+LXUWae13of2pAVIwC03KapsTkznFaL5vJQvBSmir72Q0XFzO975UhES7phEj54qmV0HANvVXc9SVvzljLiSJvJt/6eDUEyqco/rUOnneZhEarLqZch8ma+TNUbWnNO4HnNu+E31xQVjR1ADGmSpln14EFvrLD22aWyGRFufLDPxMNZ0+HYMQg2rmyDK1HFxDnk6qpvtCYTjIXcLpUPXaDF5if3wIO3mDOaTk0OwdnBav9N1/bmwmYdEzjhRnTb7A8UCAnAUSxlAYIIH3WABg2FvfhQsRJ";

      private VuforiaLocalizer vuforia;
      public TFObjectDetector tfod;

      //initializing and variables
      BNO055IMU imu;
      Orientation angles;
      private DcMotor RFMotor;
      private DcMotor LFMotor;
      private DcMotor RBMotor;
      private DcMotor LBMotor;
      private DcMotor Shooter;
      private Servo Pusher;
      private DcMotor slides;
      private DcMotor slides2;
//      private DcMotor intake;
      private Servo WobbleArmR;
      private Servo WobbleArmL;

      double RFPreviousValue = 0;
      double RBPreviousValue = 0;
      double LFPreviousValue = 0;
      double LBPreviousValue = 0;


      double one = 537.6;

      ElapsedTime t1 = new ElapsedTime();
      ElapsedTime runtime = new ElapsedTime();

      double AverageEncoderPosition;

      //  this gives you the distance and speed of motors
      double encoderSpeed(double targetPosition, double maxSpeed) {
          AverageEncoderPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
          double distance = targetPosition - AverageEncoderPosition;
          //telemetry.addData("Encoder Speed distance",distance);
          double speed = Range.clip(-distance / 500, -maxSpeed, maxSpeed); // clip the speed
          return speed;
      }

      private double encoderSpeedSide(double targetPosition, double maxSpeed) {
          double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
          double distance = targetPosition - avgEncPosition;
          telemetry.addData("difference", distance);
          double power = Range.clip(  distance / 500, -maxSpeed, maxSpeed);
          return power;
      }

      public void setForwardPower(double turnPower, double power) {
          RFMotor.setPower(-turnPower - power);
          LFMotor.setPower(turnPower - power);
          RBMotor.setPower(-turnPower - power);
          LBMotor.setPower(turnPower - power);
          telemetry.addData("turn power", turnPower);
      }

      public void setTurnPower(double turnPower, double power) {
          RFMotor.setPower(turnPower - power);
          LFMotor.setPower(-turnPower - power);
          RBMotor.setPower(turnPower - power);
          LBMotor.setPower(-turnPower - power);
          telemetry.addData("turn power", turnPower);
      }

      private void driveSideways(double turnPower, double encoderSpeedSide) {
          RFMotor.setPower(turnPower + encoderSpeedSide);
          LFMotor.setPower(-turnPower - encoderSpeedSide);
          RBMotor.setPower(turnPower - encoderSpeedSide);
          LBMotor.setPower(-turnPower + encoderSpeedSide);

      }

      double getHeading() {
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          this.imu.getPosition();
          return angles.firstAngle;
      }


      double turn(double targetAngle) {
          getHeading();
          double turnAngle = targetAngle - getHeading();
          // telemetry.addData("turnAngle", turnAngle);
          double power = Range.clip(turnAngle / 50, -0.3, 0.3);
          return power;
      }

      public void rampUp(double distance, double heading, double time, double maxSpeed) {
          double AvgEncPos = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
          double AccelerationSlope = maxSpeed / time;
          double power = t1.seconds() * AccelerationSlope;
          if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
              setForwardPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
          } else {
              if (!(Math.abs(distance - AvgEncPos) < 80)) {
                  telemetry.addData("motor is: ", "busy");
                  setForwardPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
              } else {
//                  rampUp(0,0,0,0);
                  telemetry.addData("motor is: ", "not busy");
                  RFMotor.setPower(0);
                  LFMotor.setPower(0);
                  RBMotor.setPower(0);
                  LBMotor.setPower(0);
                  setForwardPower(0, 0);

              }
          }
      }

      boolean tripLoopDoneSide = false;
      private void rampUpSide(double distance, double heading, double time, double maxSpeed) {
          double AvgEncPos = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
         telemetry.addData("Average Encoder Posistion Sideways: ", AvgEncPos);
          double AccelerationSlope = maxSpeed / time;
          double power = t1.seconds() * AccelerationSlope;
          if (Math.abs(power) < Math.abs(encoderSpeedSide(distance, maxSpeed))) {
              driveSideways(turn(heading), power);
          } else {
              if (!(Math.abs(distance - AvgEncPos) < 100) && (Math.abs(heading - getHeading()) < 5)) {
                  telemetry.addData("motor is: ", "busy");
                  driveSideways(turn(heading), encoderSpeedSide(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
              } else {
                  telemetry.addData("motor is: ", "not busy");
                  RFMotor.setPower(0);
                  LFMotor.setPower(0);
                  RBMotor.setPower(0);
                  LBMotor.setPower(0);
                  setTurnPower(0, 0);
                  tripLoopDoneSide = true;
              }
          }
      }


      public void rampUpTurn(double distance, double heading, double time, double maxSpeed) {
          double AvgEncPos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
          double AccelerationSlope = maxSpeed / time;
          double power = t1.seconds() * AccelerationSlope;
          if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
              setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
          } else {
              if (!(Math.abs(heading - getHeading()) < 15)) {
                  telemetry.addData("motor is: ", "busy");
                  setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
              } else {
                  telemetry.addData("motor is: ", "not busy");
                  RFMotor.setPower(0);
                  LFMotor.setPower(0);
                  RBMotor.setPower(0);
                  LBMotor.setPower(0);
                  setTurnPower(0, 0);

              }
          }
      }


      public void rampUpLitSide(double distance, double heading, double time, double maxSpeed){
          double AccelerationSlope = maxSpeed / time;
          double power = t1.seconds() * AccelerationSlope;
          if (Math.abs(power) < Math.abs(encoderSpeedSide(distance, maxSpeed))) {
              driveSideways(turn(heading), power);
          } else {
              telemetry.addData("motor is: ", "busy");
              driveSideways(turn(heading), encoderSpeedSide(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
          }
      }


      //Start of tensorflow program -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
      boolean Single = false;
      boolean Quad = false;
      boolean None = false;

      public void scan() {

          if (tfod != null) {
              // getUpdatedRecognitions() will return null if no new information is available since
              // the last time that call was made.
              List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
              if (updatedRecognitions != null) {
                  telemetry.addData("# Object Detected", updatedRecognitions.size());
                  if (updatedRecognitions.size() == 0) {
                      telemetry.addData("TFOD", "No Items Detected");
                      telemetry.addData("Target Zone", "A");
                      None = true;
                  } else {
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                          telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                          telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                  recognition.getLeft(), recognition.getTop());
                          telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                  recognition.getRight(), recognition.getBottom());
                          if (recognition.getLabel().equals("Single")) {
                              telemetry.addData("Target Zone", "B");
                              Single = true;
                              Quad = false;
                              None = false;
                          } else if (recognition.getLabel().equals("Quad")) {
                              telemetry.addData("Target Zone", "C");
                              Quad = true;
                              Single = false;
                              None = false;
                          } else {
                              telemetry.addData("Target Zone", "None");
                              None = true;
                              Quad = false;
                              Single = false;
                          }
                      }
                  }


              }

          }
          if (tfod != null) {
              tfod.shutdown();
          }

      }

      double linearSlideInitPos = 0;

      private double linearSlideEncSpeed(double targetPosition, double maxSpeed) {
          double distance = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
          telemetry.addData("LS distance", distance);
          double power = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
          return power;
      }

      public void initVuforia() {

          VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

          parameters.vuforiaLicenseKey = VUFORIA_KEY;
          parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

          vuforia = ClassFactory.getInstance().createVuforia(parameters);

      }

      public void initTfod() {
          int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                  "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
          tfodParameters.minResultConfidence = 0.8f;
          tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
          tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
      }

      // Trip Loops -----------------------------------------------------------------------------------------------------------
      boolean tripLoopDone = false;
      boolean EncoderPower;
      boolean tripLoop () {
          double AverageEncPower = (RFMotor.getPower() + LFMotor.getPower() + RBMotor.getPower() + LBMotor.getPower()) / 4;

          if (AverageEncPower == 0) {
              EncoderPower = false;
          } else {
              EncoderPower = true;
          }

          if (!tripLoopDone && EncoderPower) {
              tripLoopDone = true;
          }

          if (tripLoopDone && !EncoderPower) {
              RFPreviousValue = RFMotor.getCurrentPosition();
              RBPreviousValue = RBMotor.getCurrentPosition();
              LFPreviousValue = RFMotor.getCurrentPosition();
              LBPreviousValue = RFMotor.getCurrentPosition();
              return true;
          } else {
              telemetry.addData("tripLoop return:", "FALSE");
              return false;

          }
      }

      boolean tripLoopSideways(){
          if (tripLoopDoneSide){
              tripLoopDoneSide = false;
              return true;
          }

          return false;
      }

      // THIS IS WHERE THE PROGRAM STARTS---------------------------------------------------------------------------------------------------------------------------------------------------------------------/
      boolean trip1 = false;
      boolean trip2 = false;
      boolean trip3 = false;
      boolean trip4 = false;
      boolean trip5 = false;
      boolean trip6 = false;
      boolean trip7 = false;
      boolean trip8 = false;
      boolean trip9 = false;
      boolean trip10 = false;
      boolean trip11 = false;
      boolean trip12 = false;
      boolean trip13 = false;
      boolean trip14 = false;
      boolean trip15 = false;
      boolean trip16 = false;
      boolean trip17 = false;
      boolean trip18 = false;
      boolean trip19 = false;
      boolean trip20 = false;
      boolean trip21 = false;
      boolean trip22 = false;
      boolean trip23 = false;
      boolean trip24 = false;
      boolean trip25 = false;
      boolean trip26 = false;
      boolean trip27 = false;
      boolean trip28 = false;


    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("status", "initialized");

        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        //gyro stuff
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();
    }
    @Override
    public void start() {
        t1.reset();
        runtime.reset();
    }


    @Override
    public void loop(){

        // move forward
        if (!trip1) {
            rampUpSide(2*one, 0, 0.5, 0.5);
            trip1 = tripLoopSideways();
            telemetry.addData("trip", "1");
        }
        //turning
        else if(trip1 && !trip2) {
            rampUp(one*2,0,0.5,0.3);
            trip2 = tripLoop();
            telemetry.addData("trip", "2");
        }
        else if(trip2 && !trip3) {
            rampUpSide(-2*one,0,0.5,0.5);
            trip3 = tripLoopSideways();
            telemetry.addData("trip", "3");
        }
        else if(trip2 && !trip3) {
            rampUpSide(-100,0,0.5,0.5);
            trip3 = tripLoopSideways();
            telemetry.addData("trip", "3");
        }



        telemetry.addData("Runtime: ", runtime.seconds());
        telemetry.addData("tripLoopdoneSide: ", tripLoopDoneSide);
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("LFMotor encoder:", LFMotor.getCurrentPosition());
        telemetry.addData("RBMotor encoder:", RBMotor.getCurrentPosition());
        telemetry.addData("LBMotor encoder:", LBMotor.getCurrentPosition());
        telemetry.addData("heading:", getHeading());
        telemetry.addData("avg encoder:", (LBMotor.getCurrentPosition()+RBMotor.getCurrentPosition()+ LFMotor.getCurrentPosition()+RFMotor.getCurrentPosition())/4);
        telemetry.update();
    }
}
