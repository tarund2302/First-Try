package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Control.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.Control.Constants;
import org.firstinspires.ftc.teamcode.Sensors.BNO055_IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

public class Hardware implements Constants {

    HardwareMap hardwareMap;

    public AutonomousOpMode auto;

    public Telemetry telemetry;

    public BNO055_IMU imu;

    public Servo hookServo;
    public Servo markerServo;
    public Servo latchServo;

    //drivetrain
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    //extendo & actuator
    public DcMotor motorExtendo1;
    public DcMotor motorExtendo2;
    public DcMotor motorActuator;

    public Drivetrain drivetrain;

    public DcMotor[] drivetrainMotors = {motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

    public void init (HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;

        imu = new BNO055_IMU("imu", this);

        //initialize drivetrain
        motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
        motorBackRight = hardwareMap.dcMotor.get("BackRight");

        //reverse left side of drivetrain
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);


        //initialize extendo & actuator
        motorExtendo1 = hardwareMap.dcMotor.get("Extendo1");
        motorExtendo2 = hardwareMap.dcMotor.get("Extendo2");
        motorActuator = hardwareMap.dcMotor.get("Actuator");

        //initialize servos (hook & marker)
        hookServo = hardwareMap.servo.get("hook");
        markerServo = hardwareMap.servo.get("marker");

        drivetrain = new Drivetrain(Hardware.this);

    }

    public void setAuto(AutonomousOpMode auto, Telemetry telemetry)
    {
        this.auto = auto;
        this.telemetry = telemetry;

    }

    public HardwareMap getHardwareMap()
    {
        return hardwareMap;
    }

}
