package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;


//will be implemented later
public class Drivetrain implements Constants {

    private Telemetry telemetry;
    private Hardware hardware;
    private ElapsedTime runtime = new ElapsedTime();

    private Hardware robot = new Hardware();

    public Drivetrain(HardwareMap hardwareMap)
    {
        //this.hardware = hardware;//
        hardware.init(hardwareMap);

    }

    //left drive contorls
    public void leftDrive(double power)
    {
        robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);

    }

    //right drive controls
    public void rightDrive(double power)
    {
        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
    }

    //stop drivetrain
    public void stop()
    {
        leftDrive(0);
        rightDrive(0);
    }

    //reset encoders
    public void eReset()
    {
        hardware.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hardware.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void driveDistance(double speed, double leftDistance,
                              double rightDistance, double TimeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        eReset();
        newLeftTarget = hardware.motorFrontLeft.getCurrentPosition() + (int) (leftDistance * COUNTS_PER_INCH);
        newRightTarget = hardware.motorFrontRight.getCurrentPosition() + (int) (rightDistance * COUNTS_PER_INCH);

        hardware.motorFrontLeft.setTargetPosition(newLeftTarget);
        hardware.motorFrontRight.setTargetPosition(newRightTarget);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        leftDrive(Math.abs(speed));
        rightDrive(Math.abs(speed));



        stop();
    }

    public void rotateForTime(double power, double time) //Ask Ankit & Ramsey about this
    {
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            hardware.motorFrontLeft.setPower(-power);
            hardware.motorBackLeft.setPower(-power);
            hardware.motorFrontRight.setPower(power);
            hardware.motorBackRight.setPower(power);

            stopState = (System.nanoTime() - startTime) / 100000;
        }

        stop();

    }










}
