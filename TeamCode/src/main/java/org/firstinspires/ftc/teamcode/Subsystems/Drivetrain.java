package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;



public class Drivetrain implements Constants {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
   /* public Drivetrain drivetrain;*/


    private Telemetry telemetry;
    private AutonomousOpMode auto;
    private Hardware hardware;
    private Toggle toggle;
    private ElapsedTime runtime = new ElapsedTime();

    PIDController controlDrive = new PIDController(dtKP,dtKI,dtKD,dtMaxI);
    PIDController turnAngle =new PIDController(turnKP,turnKI,turnKD,turnMaxI);
    PIDController smallTurnAngle = new PIDController(turnBigKP, turnBigKI, turnBigKD,turnBigMaxI);

    public Drivetrain(Hardware hardware)
    {
        this.hardware = hardware;
        motorFrontLeft = hardware.motorFrontLeft;
        motorFrontRight = hardware.motorFrontRight;
        motorBackLeft = hardware.motorBackLeft;
        motorBackRight = hardware.motorBackRight;
        auto = hardware.auto;
        telemetry = hardware.telemetry;

        //reverse left side of drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


    }


    //left drive contorls
    public void leftDrive(double power)
    {
        hardware.motorFrontLeft.setPower(power);
        hardware.motorBackLeft.setPower(power);

    }

    //right drive controls
    public void rightDrive(double power)
    {
        hardware.motorFrontRight.setPower(power);
        hardware.motorBackRight.setPower(power);
    }

    public void drive(double leftPower, double rightPower)
    {
        hardware.motorFrontLeft.setPower(leftPower);
        hardware.motorBackLeft.setPower(leftPower);
        hardware.motorFrontRight.setPower(rightPower);
        hardware.motorBackRight.setPower(rightPower);
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
        for(DcMotor motor: hardware.drivetrainMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //send data showing encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorBackRight.getCurrentPosition());
        telemetry.update();
    }

    public void driveForward(double speed)
    {
        leftDrive(speed);
        rightDrive(speed);

        while(opModeIsActive()){
            hardware.telemetry.addData("Speed:",motorFrontLeft.getPower());
        }

    }

    public void driveDistance(double distance)
    {

        eReset();
        double counts = distanceToCounts(distance);
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeIsActive() && (stopState <= 1000))
        {
            double ePos = (hardware.motorFrontLeft.getCurrentPosition());
            double power = controlDrive.power(counts, ePos);
            hardware.telemetry.addData("Power", power);
            hardware.telemetry.addData("Distance", countsToDistance(ePos));
            hardware.telemetry.addData("Error:", Math.abs(counts-ePos));

            leftDrive(power);
            rightDrive(power);

            if(Math.abs(counts- ePos)<= distanceToCounts(DISTANCE_TOLERANCE))
            {
                hardware.telemetry.addData("Error:", Math.abs(counts-ePos));
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }

            else{
                startTime = System.nanoTime();
            }
            telemetry.update();

        }

        stop();
    }

    public void rotateForTime(double power, double time)
    {
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            hardware.motorFrontLeft.setPower(-power);
            hardware.motorBackLeft.setPower(-power);
            hardware.motorFrontRight.setPower(power);
            hardware.motorBackRight.setPower(power);

            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        }

        stop();

    }

    public void rotate(double speed)
    {
        leftDrive(-speed);
        rightDrive(speed);

        while(opModeIsActive()){
            hardware.telemetry.addData("Speed:",motorFrontLeft.getPower());
        }
    }

    public void turnAngle(double degrees)
    {

        long startTime = System.nanoTime();
        long stopState = 0;


        while(opModeIsActive() && (stopState <= 1000))
        {
            double gPos = hardware.imu.getRelativeYaw();
            double power = Math.abs(degrees) < 50 ? smallTurnAngle.power(degrees,gPos) : turnAngle.power(degrees,gPos);

            eReset();
            leftDrive(power);
            rightDrive(-power);

            if(degrees < 50)
            {
                hardware.telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                hardware.telemetry.addLine("");
                hardware.telemetry.addData("KP*error: ", smallTurnAngle.returnVal()[0]);
                hardware.telemetry.addData("KI*i: ", smallTurnAngle.returnVal()[1]);
                hardware.telemetry.addData("KD*d: ", smallTurnAngle.returnVal()[2]);
                hardware.telemetry.update();
            }

            else
            {
                hardware.telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                hardware.telemetry.addLine("");
                hardware.telemetry.addData("KP*error: ", turnAngle.returnVal()[0]);
                hardware.telemetry.addData("KI*i: ", turnAngle.returnVal()[1]);
                hardware.telemetry.addData("KD*d: ", turnAngle.returnVal()[2]);
                hardware.telemetry.update();
            }

            if (Math.abs(degrees) < 50 ? Math.abs(gPos - degrees) <= IMU_TOLERANCE : Math.abs(Math.abs(gPos) - Math.abs(degrees)) <= IMU_TOLERANCE)
            {
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }

            else
            {
                startTime = System.nanoTime();
            }

            if(System.nanoTime()/NANOSECS_PER_MILISEC-startTime/NANOSECS_PER_MILISEC>3000)
            {
                break;
            }


        }
        stop();
    }

    public void turnRelativeAngle(double degrees){
        turnAngle(hardware.imu.getYaw()+degrees);
    }

    /*public void rotateBigAngle(double angle)
    {
        eReset();
        long startTime = System.nanoTime();
        long stopState = 0;

        double degrees = angle;

        while((opModeIsActive() && (stopState <= 1000)))
        {
            double gPos = hardware.imu.getRelativeYaw();
            double power = rotateAngle.power(degrees,gPos);

            leftDrive(-power);
            rightDrive(power);

            telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
            telemetry.addLine("");
            telemetry.addData("KP*error: ", rotateAngle.returnVal()[0]);
            telemetry.addData("KI*i: ", rotateAngle.returnVal()[1]);
            telemetry.addData("KD*d: ", rotateAngle.returnVal()[2]);
            telemetry.update();

            if (Math.abs(Math.abs(gPos) - Math.abs(degrees)) <= IMU_TOLERANCE)
            {
                stopState = (System.nanoTime() - startTime) / 1000000;
            }
            else
            {
                startTime = System.nanoTime();
            }

        }
        stop();
    }*/


    public boolean opModeIsActive()
    {
        return auto.getOpModeIsActive();
    }

    public double distanceToCounts(double distance)
    {
        return (distance/WHEEL_CIRCUM)*DRIVE_GEAR_REDUCTION *NEVEREST_40_COUNTS_PER_REV;
    }

    public double countsToDistance(double counts)
    {
        return (counts*WHEEL_CIRCUM *DRIVEN_GEAR_REDUCTION)/NEVEREST_40_COUNTS_PER_REV;
    }






}
