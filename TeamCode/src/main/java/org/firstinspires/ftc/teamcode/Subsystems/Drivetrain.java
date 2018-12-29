package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Control.Direction;
import org.firstinspires.ftc.teamcode.Control.PIDController;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Sensors.MaxbotixUltrasonicSensor;

public class Drivetrain implements Constants {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private Telemetry telemetry;
    private AutonomousOpMode auto;
    private Hardware hardware;
    private BNO055_IMU imu;
   // private MaxbotixUltrasonicSensor rangeSensor;
    private Gamepad gamepad1= new Gamepad();
    private double yDirection;
    private double xDirection;
    private double leftStickY = gamepad1.left_stick_y;
    private double rightStickX = gamepad1.right_stick_x;
    private double rightStickY = gamepad1.right_stick_y;

    PIDController controlDrive = new PIDController(distanceKP,distanceKI,distanceKD,distanceMaxI);
    PIDController turnAngle =new PIDController(turnBigKP,turnBigKI,turnBigKD,turnBigMaxI);
    PIDController smallTurnAngle = new PIDController(turnKP, turnKI, turnKD,turnMaxI);
    PIDController testTurn = new PIDController(testTurnKP,testTurnKI,testTurnKD, testTurnMaxI);
    PIDController bigTestTurn = new PIDController(bigTestTurnKP,bigTestTurnKI,bigTestTurnKD, bigTestTurnMaxI);
    PIDController rangeDistance = new PIDController(rangeKP, rangeKI, rangeKD, rangeMaxI);
    PIDController turnSide = new PIDController(sideKP, sideKI, sideKD, sideMaxI);
    PIDController bigTurnSide = new PIDController(bigSideKP, bigSideKI, bigSideKD, sideMaxI);
    PIDController angleCorrection = new PIDController(angleCorrectionKP,angleCorrectionKI,angleCorrectionKD, angleCorrectionMaxI);

/*    public double frontLeftData = motorFrontLeft.getPower();
    public double frontRightData = motorFrontRight.getPower();
    public double backLeftData = motorBackLeft.getPower();
    public double backRightData = motorBackRight.getPower();*/

    public Drivetrain(Hardware hardware){
        this.hardware = hardware;
        motorFrontLeft = hardware.motorFrontLeft;
        motorFrontRight = hardware.motorFrontRight;
        motorBackLeft = hardware.motorBackLeft;
        motorBackRight = hardware.motorBackRight;
        auto = hardware.auto;
        telemetry = hardware.telemetry;
        imu = hardware.imu;
        //rangeSensor = hardware.rangeSensor;

        //reverse left side of drivetrain
        hardware.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        hardware.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    //left drive contorls
    public void leftDrive(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
    }
    //right drive controls
    public void rightDrive(double power){
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    public void driveNFS(){
        yDirection = SPEED_MUlTIPLIER * leftStickY;
        xDirection = SPEED_MUlTIPLIER * rightStickX;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;

        leftDrive(y);
        rightDrive(x);

        joystickStop(leftStickY,rightStickX);
    }
    public void invertDriveNFS(){
        yDirection = SPEED_MUlTIPLIER * -leftStickY;
        xDirection = SPEED_MUlTIPLIER * -rightStickX;

        double y = yDirection - xDirection;
        double x = yDirection + xDirection;

        leftDrive(y);
        rightDrive(x);

        joystickStop(leftStickY,rightStickX);
    }
    public void driveTank(){
        leftStickY *= SPEED_MUlTIPLIER;
        rightStickY *= SPEED_MUlTIPLIER;

        leftDrive(leftStickY);
        rightDrive(rightStickY);

        joystickStop(leftStickY,rightStickY);
    }
    public void invertDriveTank(){
        leftStickY *= SPEED_MUlTIPLIER;
        rightStickY *= SPEED_MUlTIPLIER;

        leftDrive(-leftStickY);
        rightDrive(-rightStickY);

        joystickStop(leftStickY,rightStickY);
    }

    //stop drivetrain
    public void stop(){
        leftDrive(0);
        rightDrive(0);
    }
    //stop for a certain amount of time
    public void stopTime(long time){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            stop();
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        }
    }
    //stop when joysticks are not moved
    public void joystickStop(double joystick1, double joystick2){
        if((joystick1 == 0 && joystick2 == 0)){
            stop();
        }
    }

    //reset encoders
    public void eReset(){
        for(DcMotor motor: hardware.drivetrainMotors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //send data showing encoder reset
        telemetry.addData("Status:",motorFrontLeft.getCurrentPosition());
        telemetry.update();
    }

    public void autoDrive(double speed, Direction DIRECTION){
        speed *= DIRECTION.value;
        leftDrive(speed);
        rightDrive(speed);

        while(opModeIsActive()){
            telemetry.addData("Speed:",motorFrontLeft.getPower());
        }
    }

    public void driveForTime(double power, double time, Direction DIRECTION){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            power *= DIRECTION.value;
            leftDrive(power);
            rightDrive(power);
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            telemetry.addData("Speed:",power);
            telemetry.addData("Stop state",stopState);
            telemetry.update();
        }
        stop();
    }
    public void driveDistance(double distance, Direction DIRECTION){
        eReset();
        distance*= DIRECTION.value;
        double counts = distanceToCounts(distance);
        long startTime = System.nanoTime();
        long stopState = 0;
        double initialHeading = imu.getRelativeYaw();

        while(opModeIsActive() && (stopState <= 1000))
        {
            double ePos = (motorFrontLeft.getCurrentPosition());
            double distancePower = controlDrive.power(counts, ePos);
            double angleCorrectionPower = angleCorrection.power(initialHeading,imu.getRelativeYaw());
            double leftPower = distancePower - angleCorrectionPower;
            double rightPower = distancePower + angleCorrectionPower;
            telemetry.addData("Power", distancePower);
            telemetry.addData("Distance", countsToDistance(ePos));
            telemetry.addData("Error:", Math.abs(counts-ePos));

            leftDrive(leftPower);
            rightDrive(rightPower);

            if(Math.abs(counts- ePos)<= distanceToCounts(DISTANCE_TOLERANCE))
            {
                telemetry.addData("Error:", Math.abs(counts-ePos));
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }

            else{
                startTime = System.nanoTime();
            }
            telemetry.update();
        }
        stop();
    }

/*

    public void driveTillRangeDistance(double distance){

        long startTime = System.nanoTime();
        long stopState = 0;
        double initialHeading = imu.getRelativeYaw();

        while(opModeIsActive() && (stopState <= 1000)){
            double currDistance = rangeSensor.getDistance(DistanceUnit.INCH);
            double rangePower = rangeDistance.power(distance, currDistance);
            double angleCorrectionPower = angleCorrection.power(initialHeading,imu.getRelativeYaw());
            double leftPower = rangePower - angleCorrectionPower;
            double rightPower = rangePower + angleCorrectionPower;
            telemetry.addData("Power:",rangePower);
            telemetry.addData("Distance:",currDistance);
            telemetry.addData("Error:",Math.abs(distance-currDistance));
            telemetry.addData("StopState:",stopState);
            telemetry.update();

            leftDrive(-leftPower);
            rightDrive(-rightPower);

            if(Math.abs(currDistance - distance) <= RANGE_DISTANCE_TOLERANCE){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else{
                startTime = System.nanoTime();
            }
        }
        stop();
    }
*/

    public void rotate(double speed, Direction DIRECTION){
        speed *= DIRECTION.value;
        leftDrive(-speed);
        rightDrive(speed);
        telemetry.addData("Speed:",motorFrontLeft.getPower());
        telemetry.update();
    }

    public void rotateForTime(double power, double time, Direction DIRECTION){
        long startTime = System.nanoTime();
        long stopState = 0;

        while(stopState <= time){
            power *= DIRECTION.value;
            leftDrive(-power);
            rightDrive(power);
            stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            telemetry.addData("Speed:",power);
            telemetry.addData("Stop state",stopState);
            telemetry.update();
        }
        stop();
    }

    public void turnAngle(double degrees, Direction DIRECTION){
        //imu.resetAngle();
        long startTime = System.nanoTime();
        long stopState = 0;
        degrees*=DIRECTION.value;
        double targetAngle = imu.getRelativeYaw() + degrees;

        while(opModeIsActive() && (stopState <= 1000)){
            double gPos = imu.getRelativeYaw();
            double power = Math.abs(targetAngle) < 50 ? smallTurnAngle.power(targetAngle,gPos) : turnAngle.power(targetAngle,gPos);

            leftDrive(-power);
            rightDrive(power);

            if(Math.abs(targetAngle) < 50){
                hardware.telemetry.addData("Angle:",imu.getRelativeYaw());
                hardware.telemetry.addLine("");
                hardware.telemetry.addData("KP*error: ", smallTurnAngle.returnVal()[0]);
                hardware.telemetry.addData("KI*i: ", smallTurnAngle.returnVal()[1]);
                hardware.telemetry.addData("KD*d: ", smallTurnAngle.returnVal()[2]);
                hardware.telemetry.addData("Power: ", power);
                hardware.telemetry.update();
            }
            else{
                hardware.telemetry.addData("Angle:",imu.getRelativeYaw());
                hardware.telemetry.addLine("");
                hardware.telemetry.addData("KP*error: ", turnAngle.returnVal()[0]);
                hardware.telemetry.addData("KI*i: ", turnAngle.returnVal()[1]);
                hardware.telemetry.addData("KD*d: ", turnAngle.returnVal()[2]);
                hardware.telemetry.update();
            }

            if (Math.abs(targetAngle) < 50 ? Math.abs(gPos - targetAngle) <= IMU_TOLERANCE : Math.abs(Math.abs(gPos) - Math.abs(targetAngle)) <= IMU_TOLERANCE){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else{
                startTime = System.nanoTime();
            }
        }
        stop();
    }

    public void sideTurnAngle(double degrees, String side){
        //imu.resetAngle();
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeIsActive() && (stopState <= 1000)){
            double gPos = imu.getRelativeYaw();
            double power = Math.abs(degrees) < 50 ? turnSide.power(degrees,gPos) : bigTurnSide.power(degrees,gPos);

            if(side == "left"){
                leftDrive(power);
                rightDrive(0);
            }
            else if(side == "right"){
                leftDrive(0);
                rightDrive(power);
            }
            if(Math.abs(degrees) < 50){
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", turnSide.returnVal()[0]);
                telemetry.addData("KI*i: ", turnSide.returnVal()[1]);
                telemetry.addData("KD*d: ", turnSide.returnVal()[2]);
                telemetry.addData("Power: ", power);
                telemetry.update();
            }
            else{
                telemetry.addData("Angle:",hardware.imu.getRelativeYaw());
                telemetry.addLine("");
                telemetry.addData("KP*error: ", bigTurnSide.returnVal()[0]);
                telemetry.addData("KI*i: ", bigTurnSide.returnVal()[1]);
                telemetry.addData("KD*d: ", bigTurnSide.returnVal()[2]);
                telemetry.update();
            }

            if (Math.abs(degrees) < 50 ? Math.abs(gPos - degrees) <= IMU_TOLERANCE : Math.abs(Math.abs(gPos) - Math.abs(degrees)) <= IMU_TOLERANCE){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else{
                startTime = System.nanoTime();
            }
        }
        stop();
    }

    public void testAngleCorrection(){
        double initialHeading = imu.getRelativeYaw();
        long startTime = System.nanoTime();
        long stopState = 0;

        while(opModeIsActive() && (stopState <= 1500)){
            double angleCorrectionPower = angleCorrection.power(initialHeading,imu.getRelativeYaw());

            telemetry.addData("Power:",angleCorrectionPower);
            telemetry.addData("Heading:",imu.getRelativeYaw());
            telemetry.addData("Error:", Math.abs(initialHeading-imu.getRelativeYaw()));
            telemetry.update();

            leftDrive(-angleCorrectionPower);
            rightDrive(angleCorrectionPower);

            if(Math.abs(imu.getRelativeYaw() - initialHeading) <= IMU_TOLERANCE){
                stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
            }
            else{
                startTime = System.nanoTime();
            }
        }
        //stop();
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

/*    public double[] getData(){
        double dtData[] = {frontLeftData,frontRightData,backLeftData,backRightData};
        return dtData;
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