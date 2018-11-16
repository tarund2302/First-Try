package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;//
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp_Test")
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


        //initilization positions
        hookServo.setPosition(HOOK_DOWN_POSITION);
        motorExtendo.setPower(0); //stops motor

        waitForStart();

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






        }
    }



}
