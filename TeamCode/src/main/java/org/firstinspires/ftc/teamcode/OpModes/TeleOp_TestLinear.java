package org.firstinspires.ftc.teamcode.OpModes;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;//
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//v2.1 code

@TeleOp(name = "TeleOp_Test")
@Disabled
public class TeleOp_TestLinear extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //declare motors
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorExtendo;

    //declare servos
    private Servo hookServo;
    private Servo markerServo;

    //servo positions
    private static final double HOOK_DOWN_POSITION = -1;
    private static final double HOOK_UP_POSITION = 0;
    private static final double MARKER_UP_POSITION = 0;
    private static final double MARKER_DOWN_POSITION = 0.7;


    @Override
    public void runOpMode () throws InterruptedException
    {
        //initialize drivetrain motors
        motorFrontLeft = hardwareMap.dcMotor.get("front_left_wheel");
        motorFrontRight = hardwareMap.dcMotor.get("front_right_wheel");
        motorBackLeft = hardwareMap.dcMotor.get("back_left_wheel");
        motorBackRight = hardwareMap.dcMotor.get("back_right_wheel");

        //initialize extendo
        motorExtendo = hardwareMap.dcMotor.get("extendo");

        //reverse left side of the drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        //initialize servos
        hookServo = hardwareMap.servo.get("hook");
        markerServo = hardwareMap.servo.get("marker");

        //starting positions for servos
        hookServo.setPosition(HOOK_DOWN_POSITION);
        markerServo.setPosition(MARKER_UP_POSITION);

        //stop extendo motor
        motorExtendo.setPower(0); //stops motor

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {

            //controls for drivetrain
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_y);



            //controls for extendo
            motorExtendo.setPower(gamepad1.right_trigger); //extend
            motorExtendo.setPower(-gamepad1.left_trigger); //retracts



            if(gamepad1.a) //hook down
            {
                hookServo.setPosition(HOOK_DOWN_POSITION);
            }

            if(gamepad1.x) //hook up
            {
                hookServo.setPosition(HOOK_UP_POSITION);
            }

            // Show the elapsed game time and wheel power. 
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);
            telemetry.update();




        }
    }



}
