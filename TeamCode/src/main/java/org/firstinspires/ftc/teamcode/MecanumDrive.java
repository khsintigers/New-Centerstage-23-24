package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MecanumDrive", group="Linear Opmode")

public class MecanumDrive extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor lifter_motor = null;
    private DcMotor fifth_motor = null;
    private DcMotor sixth_motor = null;
    public DcMotor extend= null;
    public Servo test_servo = null;
    public Servo claw_height = null;
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

        //fifth_motor  = hardwareMap.get(DcMotor.class, "FifthMotor");
        //sixth_motor  = hardwareMap.get(DcMotor.class, "SixthMotor");
        //extend = hardwareMap.get(DcMotor.class, "lifter");
        // ball_pusher = hardwareMap.get(Servo.class, "BallPusher");
        // POV Mode uses left stick to go forward, and right stick to turn.5
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
        // get a reference to touch sensor.
        //touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        //touchRight = hardwareMap.touchSensor.get("TouchRight");

        int lifterBottom = 0;

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        final double maxPower = 0.5;
        // set power to zero to avoid a FTC bug
        right_front.setPower(0);
        left_front.setPower(0);
        left_rear.setPower(0);
        right_rear.setPower(0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            mechanum();

            //motor rotation value

            telemetry.addData("lift: ","running" );
            telemetry.update();


        }

    }




    public void mechanum(){
        final double maxPower = 1.0;   //0.5

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













/*
    public void Mechanum(){
        final double maxPower = 1;

        double joy1Y = -gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        rf.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        lf.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        rr.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        lr.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
    }

    */
private void resetAngle() {
    lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    globalAngle = 0;
}

    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.
    private double getAngle() {
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
    private double checkDirection() {
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
        if (angle > 0) {
            direction = -1;
        }
        while ((Math.abs(angle) - Math.abs(getAngle()) > tolerance)) {
            left_front.setPower(motorPower * direction);
            right_front.setPower(-1 * motorPower * direction);
            left_rear.setPower(motorPower * direction);
            right_rear.setPower(-1 * motorPower * direction);

            telemetry.addData("Wanted Angle", angle);
            telemetry.addData("Curr Angle", getAngle());
            telemetry.update();
        }
    }
}
