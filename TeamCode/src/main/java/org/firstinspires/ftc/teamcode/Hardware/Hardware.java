package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Sensors.MaxbotixUltrasonicSensor;
import org.firstinspires.ftc.teamcode.Sensors.RevBlinkinLEDDriver;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.GoldFinder;

public class Hardware implements Constants {

    HardwareMap hardwareMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public BNO055_IMU imu;

//    public MaxbotixUltrasonicSensor rangeSensor;

  //  RevBlinkinLEDDriver blinkinLedDriver;

    /*public Servo hookServo;
    public Servo markerServo;*/
    public Servo servo;

    //drivetrain
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;
    public DcMotor winch;

    //extendo & actuator
   /* public DcMotor motorExtendo1;
    public DcMotor motorExtendo2;
    public DcMotor motorActuator;*/

    public Drivetrain drivetrain;

    public GoldFinder gold;

    public DcMotor[] drivetrainMotors;

    public void init (HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;


        //initialize drivetrain
        /*motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        //motorBackRight = hardwareMap.dcMotor.get("BackRight");*/
        winch = hardwareMap.get(DcMotor.class,"winch");
        motorFrontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class,"BackRight");

        DcMotor[] tempMotorArray = {motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};
        drivetrainMotors = tempMotorArray;

        //reverse left side of drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = new BNO055_IMU("imu", this);
        //blinkinLedDriver = hardwareMap.get(RevBlinkinLEDDriver.class, "blinkin");
        drivetrain = new Drivetrain(this);
        gold = new GoldFinder(this);
        //rangeSensor = new MaxbotixUltrasonicSensor(hardwareMap.analogInput.get("rangeSensor"));

      /*  //initialize extendo & actuator
        motorExtendo1 = hardwareMap.dcMotor.get("Extendo1");
        motorExtendo2 = hardwareMap.dcMotor.get("Extendo2");
        motorActuator = hardwareMap.dcMotor.get("Actuator");
*/
        //initialize servos (hook & marker)
        /*hookServo = hardwareMap.servo.get("hook");
        markerServo = hardwareMap.servo.get("marker");*/
        /* latchServo = hardwareMap.servo.get("drop");*/


    }

    public void setAuto(AutonomousOpMode auto){
        this.auto = auto;
    }

    public void setTelemetry (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public HardwareMap getHardwareMap()
    {
        return hardwareMap;
    }

}
