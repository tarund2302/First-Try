package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hardware {

    HardwareMap hardwareMap;

//    public Telemetry telemetry;

    public Servo hookServo;

    public Servo markerServo;

    //drivetrain
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    //extendo & actuator
    public DcMotor motorExtendo1;
    public DcMotor motorExtendo2;
    public DcMotor motorActuator;


    public void init (HardwareMap hardwareMap)
    {
        this.hardwareMap = hardwareMap;

            //initialize drivetrain
            motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft");
            motorFrontRight = hardwareMap.dcMotor.get("FrontRight");
            motorBackLeft = hardwareMap.dcMotor.get("BackLeft");
            motorBackRight = hardwareMap.dcMotor.get("BackRight");




            //initialize extendo & actuator
            motorExtendo1 = hardwareMap.dcMotor.get("Extendo1");
            motorExtendo2 = hardwareMap.dcMotor.get("Extendo2");
            motorActuator = hardwareMap.dcMotor.get("Actuator");

            //initialize servos (hook & marker)
            hookServo = hardwareMap.servo.get("hook");
            markerServo = hardwareMap.servo.get("marker");



    }





}
