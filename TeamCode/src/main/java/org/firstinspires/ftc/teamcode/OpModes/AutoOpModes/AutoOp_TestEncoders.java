package org.firstinspires.ftc.teamcode.OpModes.AutoOpModes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@Autonomous(name = "AutoOp_Encoders")
public class AutoOp_TestEncoders extends LinearOpMode implements Constants
{

    private Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private Drivetrain drivetrain = new Drivetrain(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        drivetrain.eReset();

        /*robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        //send data showing encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                            robot.motorFrontLeft.getCurrentPosition(),
                            robot.motorBackLeft.getCurrentPosition(),
                            robot.motorFrontRight.getCurrentPosition(),
                            robot.motorBackRight.getCurrentPosition());
        telemetry.update();

        robot.hookServo.setPosition(HOOK_UP_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);

        waitForStart();

        robot.motorActuator.setPower(1);
        sleep(1260);
        drivetrain.rotateForTime(TURN_SPEED, 1000);
        robot.hookServo.setPosition(HOOK_DOWN_POSITION);
        robot.motorActuator.setPower(-1);
        sleep(1500);

        drivetrain.rotateForTime(- TURN_SPEED, 1000);
        drivetrain.driveDistance(DRIVE_SPEED, 24, 24,5.0);
        robot.markerServo.setPosition(MARKER_DOWN_POSITION);
        robot.markerServo.setPosition(MARKER_UP_POSITION);
        drivetrain.rotateForTime(TURN_SPEED, 750);
        drivetrain.driveDistance(DRIVE_SPEED, 48, 48,5.0);

        /*EncoderDrive(DRIVE_SPEED, 24, 24, 5.0);
        EncoderDrive(TURN_SPEED, -24, 24, 2.0);
        EncoderDrive(DRIVE_SPEED, 48, 48, 5.0);*/



    }

    public void EncoderDrive(double speed, double leftInches,
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






    } //encoder drive method ends


} //main

