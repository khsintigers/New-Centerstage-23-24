package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;


@Autonomous()
public class TestRun extends LinearOpMode{

    /* Declare OpMode members. */
    private DcMotor left_front   = null;
    private DcMotor left_rear = null;
    private DcMotor right_front  = null;
    private DcMotor right_rear  = null;
    private DcMotor swing_motor = null;
    private DcMotor left_lift  = null;
    private DcMotor right_lift  = null;
    public DcMotor extend= null;
    public Servo pixel_claw = null;
    public Servo pixel_sleeve = null;


    public void runOpMode() {
        // Initialize the drive system variables.
        left_rear  = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
        left_front  = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front = hardwareMap.get(DcMotor.class, "FrontRight");
        swing_motor  = hardwareMap.get(DcMotor.class, "SwingMotor");
        left_lift  = hardwareMap.get(DcMotor.class, "LeftLift");
        right_lift  = hardwareMap.get(DcMotor.class, "RightLift");
        pixel_claw= hardwareMap.get(Servo.class, "PixelClaw");
        pixel_sleeve= hardwareMap.get(Servo.class, "PixelSleeve");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);

        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_rear.setDirection(DcMotorSimple.Direction.REVERSE);
        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)\
            while(opModeIsActive()) {
                double speed = 0.5;
                if(gamepad1.x) {
                    speed = 0.75;
                }
                if(gamepad1.a) {
                    speed = 0.1;
                }
                if(gamepad1.b) {
                    speed = 0.5;
                }
                if(gamepad1.y) {
                    speed = 1;
                }
                if(gamepad1.right_bumper) {
                    left_rear.setPower(speed);
                    right_rear.setPower(speed);
                    left_front.setPower(speed);
                    right_front.setPower(speed);
                    sleep(2000);
                    stopAllMotion();
                }


                telemetry.addData("Path", "Complete");
                telemetry.update();
            }



            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        sleep(1000);  // pause to display final telemetry message.
    }
//    @Override
//    public void init_loop() {
//    }

//    @Override
//    public void start() {
//        //visionPortal.stopStreaming();
//    }

    /*
    Add back in later, figure out how to thread loops
    add into the encoderDrive Loop

    @Override
    public void loop() {
        telemetry.addData("Identified", drawRectangleProcessor.getSelection());
        telemetry.addData( "Left Avg: ",drawRectangleProcessor.getLeftAvg());
        telemetry.addData( "Middle Avg: ",drawRectangleProcessor.getMiddleAvg());
        telemetry.addData( "Right Avg: ",drawRectangleProcessor.getRightAvg());
        telemetry.update();

    }
    */
    public void stopAllMotion() {
        left_rear.setPower(0);
        right_rear.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }
    public boolean wheelsInMotion() {
        return left_rear.isBusy() && right_rear.isBusy() && left_front.isBusy() && right_front.isBusy();
    }


}
