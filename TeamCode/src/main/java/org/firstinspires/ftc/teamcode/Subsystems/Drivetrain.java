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
    public Drivetrain drivetrain;


    private Telemetry telemetry;
    private AutonomousOpMode auto;
    private Hardware hardware;
    private Toggle toggle;
    private ElapsedTime runtime = new ElapsedTime();
    /*private Gamepad gamepad;*/
    PIDController controlDrive = new PIDController(dtKP,dtKI,dtKD,dtMaxI);
    PIDController turnAngle =new PIDController(rotateKP,rotateKI,rotateKD,rotateMaxI);
    PIDController smallTurnAngle = new PIDController(rotateBigKP, rotateBigKI, rotateBigKD,rotateBigMaxI);


    //private Hardware robot = new Hardware();//

    public Drivetrain(Hardware hardwareMap)
    {
        this.hardware = hardwareMap;
        motorFrontLeft = hardware.motorFrontLeft;
        motorFrontRight = hardware.motorFrontRight;
        motorBackLeft = hardware.motorBackLeft;
        motorBackRight = hardware.motorBackRight;
        auto = hardware.auto;
        //hardware.init(hardwareMap);//
        //telemetry = hardware.telemetry;

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

    /*public void invertDrive(double power, boolean button){
        toggle.toggle(boolean );
    }*/

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
                drivetrain.motorFrontLeft.getCurrentPosition(),
                drivetrain.motorBackLeft.getCurrentPosition(),
                drivetrain.motorFrontRight.getCurrentPosition(),
                drivetrain.motorBackRight.getCurrentPosition());
        telemetry.update();
    }

    public void driveForward(double speed)
    {
        leftDrive(speed);
        rightDrive(speed);
    }

    public void driveDistance(/*double speed, double leftDistance,
                              double rightDistance, */ double distance/*, double TimeoutS*/)
    {
        //int newLeftTarget;
        //int newRightTarget;
        //int newTarget;


        eReset();
        double counts = (distance/WHEEL_CIRCUM)*DRIVE_GEAR_REDUCTION*NEVEREST_40_COUNTS_PER_REV;
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeIsActive() && (stopState <= 1000))
        {
            double /*avg*/ ePos = (hardware.motorFrontLeft.getCurrentPosition())/*+(hardware.motorBackLeft.getCurrentPosition()))/2*/;
            double power = controlDrive.power(counts, ePos/*avg*/);
            telemetry.addData("Power", power);
            telemetry.addData("Distance", countsToDistance(/*avg*/ ePos));

            leftDrive(power);
            rightDrive(power);

            if(Math.abs(counts-/*avg*/ ePos)<= distanceToCounts(DISTANCE_TOLERANCE))
            {
                telemetry.addData("Error:", Math.abs(counts-/*avg*/ ePos));
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
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", smallTurnAngle.returnVal()[0]);
                telemetry.addData("KI*i: ", smallTurnAngle.returnVal()[1]);
                telemetry.addData("KD*d: ", smallTurnAngle.returnVal()[2]);
                telemetry.update();
            }

            else
            {
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", turnAngle.returnVal()[0]);
                telemetry.addData("KI*i: ", turnAngle.returnVal()[1]);
                telemetry.addData("KD*d: ", turnAngle.returnVal()[2]);
                telemetry.update();
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
        /*rotateToAbsoluteAngle(hardware.imu.getYaw()+degrees);*/
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
