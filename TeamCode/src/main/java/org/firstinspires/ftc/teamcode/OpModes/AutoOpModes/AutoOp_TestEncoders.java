package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

//v2 auto code made by Tarun Dasari, Rookie Programmer of 3846 Maelstrom
//last update: November 21, 2018

@Autonomous(name = "AutoOp_Encoders")
public class AutoOp_TestEncoders extends LinearOpMode implements Constants
{

    private Hardware robot = new Hardware();
    /*private ElapsedTime runtime = new ElapsedTime();*/
    //private Drivetrain drivetrain = new Drivetrain(Hardware hardwareMap);

    public boolean getOpModeIsActive()
    {
        return opModeIsActive();
    }

    public Telemetry getTelemetry()
    {
        return telemetry;
    }


    @Override
    public void runOpMode() throws InterruptedException {

        //robot.setAuto(this, telemetry);

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.drivetrain.eReset();

        /*robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/




        robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        waitForStart();

        //unlatch
        robot.motorActuator.setPower(1);
        sleep(1260);
        robot.drivetrain.rotateForTime(TURN_SPEED, 1000);
        robot.hookServo.setPosition(HOOK_DOWN_POSITION);
        robot.motorActuator.setPower(-1);
        sleep(1500);
        robot.drivetrain.rotateForTime(- TURN_SPEED, 1000);

        //drive towards crater
        /*robot.drivetrain.driveDistance(36,4.0);*/
        robot.drivetrain.driveDistance(36);

        //extend marker lift and drop marker
        robot.motorExtendo1.setPower(1);
        robot.motorExtendo2.setPower(1);
        sleep(1000);

        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);
        robot.markerServo.setPosition(MARKER_DOWN_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        //retract marker lift
        robot.motorExtendo1.setPower(-1);
        robot.motorExtendo2.setPower(-1);
        sleep(1000);
        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);

        //rotate towards crater from front of bot
        /*robot.drivetrain.rotateForTime(TURN_SPEED, 750);*/
        robot.drivetrain.rotateAngle(30);
        robot.drivetrain.rotateAngle(90);
        robot.drivetrain.rotateAngle(90);
        /*robot.drivetrain.rotateBigAngle(180);*/

        //drive towards crater
        robot.drivetrain.driveDistance(60);
        /*robot.drivetrain.driveDistance(12,4.0);*/

        //extend to pass over crater rim
        robot.motorExtendo1.setPower(1);
        robot.motorExtendo2.setPower(1);
        sleep(1000);
        robot.motorExtendo1.setPower(0);
        robot.motorExtendo2.setPower(0);

        /*EncoderDrive(DRIVE_SPEED, 24, 24, 5.0);
        EncoderDrive(TURN_SPEED, -24, 24, 2.0);
        EncoderDrive(DRIVE_SPEED, 48, 48, 5.0);*/

        /*
        //testing for gyro
        robot.drivetrain.rotateAngle(90);
        robot.drivetrain.rotateAngle(-90);
        robot.drivetrain.stop();
        sleep(5000);
        robot.drivetrain.rotateBigAngle(180);
        robot.drivetrain.rotateBigAngle(-180);
        robot.drivetrain.stop();

        //testing for encoders
        robot.drivetrain.driveDistance(12);
        robot.drivetrain.driveDistance(-12);
        robot.drivetrain.driveDistance(40);
        robot.drivetrain.driveDistance(-40);
        */
    }

    /*public void EncoderDrive(double speed, double leftInches,
                             double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        if(opModeIsActive())
        {
            newLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.motorFrontLeft.setTargetPosition(newLeftTarget);
            robot.motorBackLeft.setTargetPosition(newLeftTarget);

            robot.motorFrontRight.setTargetPosition(newRightTarget);
            robot.motorBackRight.setTargetPosition(newRightTarget);

            //Turn on RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //reset timeout time and start motion
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorBackLeft.isBusy() &&
                            robot.motorFrontRight.isBusy() &&robot.motorBackRight.isBusy()))

            {
                //Display data at DS
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running to %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(), robot.motorBackLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition(), robot.motorBackRight.getCurrentPosition());

                telemetry.update();
            } //telemetry loop ends

            //stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackRight.setPower(0);

            //turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        } //opModeActive statement ends






    } //encoder drive method ends*/


} //main

