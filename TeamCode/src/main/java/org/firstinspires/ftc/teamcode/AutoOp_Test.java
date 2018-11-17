package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoOp_Test")
public class AutoOp_Test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorExtendo;

    //declare servos
    Servo hookServo;
    Servo markerServo;

    private static final double HOOK_DOWN_POSITION = -1;
    private static final double HOOK_UP_POSITION = 0;
    private static final double MARKER_UP_POSITION = 0;
    private static final double MARKER_DOWN_POSITION = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        //initialize motors
        motorFrontLeft = hardwareMap.dcMotor.get("front_left_wheel");
        motorFrontRight = hardwareMap.dcMotor.get("front_right_wheel");
        motorFrontLeft = hardwareMap.dcMotor.get("back_left_wheel");
        motorFrontLeft = hardwareMap.dcMotor.get("back_right_wheel");
        motorExtendo = hardwareMap.dcMotor.get("extendo");

        //reverse left side of the drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //initialize servos
        hookServo = hardwareMap.servo.get("hook");
        markerServo = hardwareMap.servo.get("marker");

        //starting positions for servos
        hookServo.setPosition(HOOK_UP_POSITION);
        markerServo.setPosition(MARKER_UP_POSITION);



        waitForStart();
        runtime.reset();

        //unlatch
        motorExtendo.setPower(1);
        sleep(1500);
        hookServo.setPosition(HOOK_DOWN_POSITION);

        //drive to depot
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        sleep(5000);

        //stop
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        //drop marker
        markerServo.setPosition(MARKER_DOWN_POSITION);
        sleep(1000);
        markerServo.setPosition(MARKER_UP_POSITION);

        //turn towards crater
        motorFrontLeft.setPower(-0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(-0.5);
        motorBackRight.setPower(0.5);
        sleep(500);

        //drive towards crater
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        sleep(10000);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
        telemetry.update();






    }
}
