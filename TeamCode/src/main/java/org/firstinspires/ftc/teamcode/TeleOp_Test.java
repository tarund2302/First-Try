package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;//
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Test", group = "OpMode")
public class TeleOp_Test extends LinearOpMode
{

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor motorExtendo;

    private Servo hookServo;

    private static final double HOOK_DOWN_POSITION = 1;
    private static final double HOOK_UP_POSITION = 0;

    @Override
    public void runOpMode () throws InterruptedException
    {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorExtendo = hardwareMap.dcMotor.get("motorExtendo");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        hookServo = hardwareMap.servo.get("hookServo");

//        hookServo.getPosition(HOOK_DOWN_POSITION);
//        hookServo.getPosition(HOOK_UP_POSITION);

        //initilization positions
        hookServo.setPosition(HOOK_DOWN_POSITION);
        motorExtendo.setPower(0); //stops motor

        waitForStart();

        while(opModeIsActive())
        {
            //controls for moving fowarding and backward
            if(gamepad1.left_stick_y > 0) //drive forward
            {
                motorFrontLeft.setPower(gamepad1.left_stick_y);
                motorFrontRight.setPower(gamepad1.left_stick_y);
                motorBackLeft.setPower(gamepad1.left_stick_y);
                motorBackRight.setPower(gamepad1.left_stick_y);
            }

            else if(gamepad1.left_stick_y < 0) //drive backward
            {
                motorFrontLeft.setPower(-gamepad1.left_stick_y);
                motorFrontRight.setPower(-gamepad1.left_stick_y);
                motorBackLeft.setPower(-gamepad1.left_stick_y);
                motorBackRight.setPower(-gamepad1.left_stick_y);
            }


            //controls for turning
            if(gamepad1.right_stick_x > 0) //turn right
            {
                motorFrontLeft.setPower(gamepad1.right_stick_x);
                motorFrontRight.setPower(-gamepad1.right_stick_x);
                motorBackLeft.setPower(gamepad1.right_stick_x);
                motorBackRight.setPower(-gamepad1.right_stick_x);
            }

            else if(gamepad1.right_stick_x < 0) //turn left
            {
                motorFrontLeft.setPower(-gamepad1.right_stick_x);
                motorFrontRight.setPower(gamepad1.right_stick_x);
                motorBackLeft.setPower(-gamepad1.right_stick_x);
                motorBackRight.setPower(gamepad1.right_stick_x);
            }


            //controls for extendo
            if(gamepad1.right_trigger > 0) //extends
            {
                motorExtendo.setPower(gamepad1.right_trigger);
            }

            else if(gamepad1.left_trigger > 0) //retracts
            {
                motorExtendo.setPower(-gamepad1.left_trigger);
            }


            if(gamepad1.a) //hook down
            {
                hookServo.setPosition(HOOK_DOWN_POSITION);
            }

            if(gamepad1.x) //hook up
            {
                hookServo.setPosition(HOOK_UP_POSITION);
            }


//            if(gamepad1.a)
//            {
//                hookServo.getPosition(HOOK_DOWN_POSITION);
//            }
//
//            if(gamepad1.x)
//            {
//                hookServo.getPosition(HOOK_UP_POSITION);
//            }



            idle();
        }
    }



}
