package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CenterStage-Driver", group="Linear Opmode")

public class CenterStage_Driver extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor swing_motor = null;
    private DcMotor left_lift  = null;
    private DcMotor right_lift  = null;
    public DcMotor extend= null;
    //public Servo pixel_claw = null;
    public Servo pixel_arm = null;
    public Servo left_gate = null;
    public Servo right_gate = null;
    private double maxPower = 1;

    public double speedFactor = 1.0;  // 100% speed
    BNO055IMU imu; //gyroscope
    TouchSensor touchLeft, touchRight;
    boolean touchedTriggered;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);
        left_rear    = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear   = hardwareMap.get(DcMotor.class, "RearRight");
        left_front   = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front  = hardwareMap.get(DcMotor.class, "FrontRight");

        swing_motor  = hardwareMap.get(DcMotor.class, "SwingMotor");
        left_lift  = hardwareMap.get(DcMotor.class, "LeftLift");
        right_lift  = hardwareMap.get(DcMotor.class, "RightLift");
        //sixth_motor  = hardwareMap.get(DcMotor.class, "SixthMotor");
        //extend = hardwareMap.get(DcMotor.class, "lifter");
      //  pixel_claw= hardwareMap.get(Servo.class, "PixelClaw");
        left_gate = hardwareMap.get(Servo.class, "LeftGate");
        right_gate = hardwareMap.get(Servo.class, "RightGate");
        pixel_arm= hardwareMap.get(Servo.class, "PixelArm");

        // POV Mode uses left stick to go forward, and right stick to turn.5
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        // get a reference to touch sensor.
        //touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        //touchRight = hardwareMap.touchSensor.get("TouchRight");


        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swing_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        swing_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_rear.setDirection(DcMotorSimple.Direction.REVERSE);
        left_rear.setDirection(DcMotorSimple.Direction.REVERSE);

        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        // set power to zero to avoid a FTC bug
        right_front.setPower(0);
        left_front.setPower(0);
        left_rear.setPower(0);
        swing_motor.setPower(0);
        left_lift.setPower(0);
        right_lift.setPower(0);
        pixel_arm.setPosition(0.5);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            mechanum();


            //swing motor
            if (gamepad2.left_trigger==1.0) {
                swing_motor.setPower(-0.3);
        //        pixel_claw.setPosition(1);
            }
            else {
                if (gamepad2.left_bumper) {
                    swing_motor.setPower(0.3);

                }
                else {
                    swing_motor.setPower(0);
                }
            }

            if (gamepad2.right_trigger==1.0) {
                left_lift.setPower(0.2);
                right_lift.setPower(0.2);
            }
            else {
                if (gamepad2.right_bumper) {
                    left_lift.setPower(-0.2);
                    right_lift.setPower(-0.2);
                }
                else {
                    left_lift.setPower(0);
                    right_lift.setPower(0);
                }
            }


            //Pixel sleeve
            if (gamepad2.x) {
                pixel_arm.setPosition(pixel_arm.getPosition() + 0.01);
            }
            if (gamepad2.b) {
                left_gate.setPosition(0.5);//open
                right_gate.setPosition(0.3);
            }

            if (gamepad2.y) {
                pixel_arm.setPosition(pixel_arm.getPosition() - 0.01);
            }
            if (gamepad2.a) {
                left_gate.setPosition(0.7); //close
                right_gate.setPosition(0);
            }

            if(gamepad1.right_trigger == 1.0) {
                maxPower = 0.75;
            } else if (gamepad1.right_bumper) {
                maxPower = 0.50;
            } else if (gamepad1.left_bumper) {
                maxPower = 0.25;
            } else if (gamepad1.left_trigger == 1.0) {
                maxPower = 0.1;
            }
/*
            if (gamepad1.dpad_down) {
                left_front.setPower(1 * speedFactor);
                right_front.setPower(1 * speedFactor);
                left_rear.setPower(1 * speedFactor);
                right_rear.setPower(1 * speedFactor);
            }
            else {
                if (gamepad1.dpad_down) {
                    left_front.setPower(-1 * speedFactor);
                    right_front.setPower(-1 * speedFactor);
                    left_rear.setPower(-1 * speedFactor);
                    right_rear.setPower(-1 * speedFactor);
                }
                else {
                    left_front.setPower(0);
                    right_front.setPower(0);
                    left_rear.setPower(0);
                    right_rear.setPower(0);
                }
            }

            // adjust speed of driving motors
            if (gamepad1.y) {
                speedFactor = 1.0;
            }
            if (gamepad1.x) {
                speedFactor = 0.75;
            }
            if (gamepad1.b) {
                speedFactor = 0.5;
            }
            if (gamepad1.a) {
                speedFactor = 0.25;
            }
*/
            telemetry.addData("lift: ","running" );
            telemetry.addData("position", pixel_arm.getPosition());
            telemetry.update();


        }

    }




    public void mechanum(){

        double joy1Y = gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        left_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        right_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        left_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        right_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));


        //telemetry.addData("LF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        //telemetry.addData("RF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        //telemetry.addData("LR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        //telemetry.addData("RR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        //telemetry.update();
    }






}








