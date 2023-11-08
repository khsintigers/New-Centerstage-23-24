package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="FoundationMove", group="Skyline")
//@Disabled

public class FoundationMove extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //map to config file
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor fifthMotor = null;
    DcMotor sixthMotor = null;
   // Servo RightLatch = null;
   // Servo LeftLatch = null;
   // TouchSensor touchLeft, touchRight;

    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
   // ColorSensor colorSensor, bottomSensor;
   // DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        sleep(500);
        //link motors, servos, etc
        leftRear = hardwareMap.get(DcMotor.class, "RearLeft");
        rightRear = hardwareMap.get(DcMotor.class, "RearRight");
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
      //  RightLatch  = hardwareMap.get(Servo.class, "RightLatch");
      //  LeftLatch = hardwareMap.get(Servo.class, "LeftLatch");
      //  touchLeft = hardwareMap.touchSensor.get("TouchLeft");
      //  touchRight = hardwareMap.touchSensor.get("TouchRight");
        //setup gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port on a
        // Core Device Interface Module, configured to be a sensor of type "AREV Expansion Hub IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // make sure the imu gyro is calibrated before continuing.
        //check out file SensorREVColorDistance for more info
      //  colorSensor = hardwareMap.colorSensor.get("ColorSensor");
       // bottomSensor = hardwareMap.colorSensor.get("BottomSensor");
        // get a reference to the distance sensor that shares the same name.
       // distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");
        // IMPORTANT-In robot config use odd numbered ports for each digital device

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        runtime.reset();

        int slideDistance = 2500; //distance to strafe to the left
        int backUpDistance = 200; //distance to back away from the wall b4 strafing to the left
        int clearWallDistance = 400; //forward distance that determines clearing the wall
        int wallPos = 0, slidePos;
        //move ahead for 4 secs
        double speed = .6;
        boolean wallstruck = false;
        leftFront.setPower(speed/2);
        rightFront.setPower(speed/2);
        leftRear.setPower(speed/2);
        rightRear.setPower(speed/2);
        sleep(2000);
//stop open latch
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        //RightLatch.setPosition(0.0);
        //LeftLatch.setPosition(0.0);

        //sleep(500);


        //leftFront.setPower(-speed);
        //rightFront.setPower(-speed);
        //leftRear.setPower(-speed);
        //rightRear.setPower(-speed);
        //while(!(wallstruck)) {
        //    if (touchRight.isPressed() || touchLeft.isPressed()) {
        //        wallstruck = true;
        //    }
        //}
        //turn
       //resetAngle();
       turnAngle(-90, 2, .5);

//unlatch
        //RightLatch.setPosition(0.5);
        //LeftLatch.setPosition(0.5);

        //straighten
        //leftFront.setPower(speed);
        //rightFront.setPower(speed);
        //leftRear.setPower(speed);
        //rightRear.setPower(speed);
    //sleep(1000);





        //move backword
/*
        while (!(bottomSensor.red() > 2*bottomSensor.blue())){
            leftFront.setPower(-speed);
            rightFront.setPower(-speed);
            leftRear.setPower(-speed);
            rightRear.setPower(-speed);

            telemetry.addData("Red: ", bottomSensor.red());
            telemetry.addData("Green: ", bottomSensor.green());
            telemetry.addData("Blue: ", bottomSensor.blue());
            telemetry.addData("Luminosity: ", bottomSensor.alpha());
            telemetry.addData("Argb: ", bottomSensor.argb());
            telemetry.update();
        }
*/



        //leftFront.setPower(0);
        //rightFront.setPower(0);
        //leftRear.setPower(0);
        //rightRear.setPower(0);



        sleep(1000);






    }


    // *******  GYROSCOPE Methods   *********

    // Resets the cumulative angle tracking to zero.
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }


    // See if we are moving in a straight line and if not return a power correction value.
    // @return Power adjustment, + is adjust left - is adjust right.
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void turnAngle(double angle, int tolerance, double motorPower) {
        //tolerance: how much angle can be off by
        //motorPower: how fast to turn
        int direction = 1; //+ is clockwise turning
        resetAngle();
        if (angle < 0) {
            direction = -1;
        }
        while ((Math.abs(angle) - Math.abs(getAngle()) > tolerance)) {
            leftFront.setPower(motorPower * direction);
            rightFront.setPower(-1 * motorPower * direction);
            leftRear.setPower(motorPower * direction);
            rightRear.setPower(-1 * motorPower * direction);

            telemetry.addData("Wanted Angle", angle);
            telemetry.addData("Curr Angle", getAngle());
            telemetry.update();
        }
    }

    //fires ball



}