package org.firstinspires.ftc.teamcode.Control;

public interface Constants {

    //hook positions
    double HOOK_UP_POSITION = 0;
    double HOOK_DOWN_POSITION = 1;

    //marker positions
    double MARKER_UP_POSITION = 1;
    double MARKER_DOWN_POSITION = -1;

    double NEVEREST_20_COUNTS_PER_REV = 560; //extendo motor ticks
    double NEVEREST_40_COUNTS_PER_REV = 1120; //drivetrain motor ticks
    double ORBITAL_3_7_COUNTS_PER_REV = 103; //actuator motor ticks

    double WHEEL_DIAMETER_INCHES = 4.0;
    double DRIVE_GEAR_REDUCTION = 0.5; //gear ratio (driven gear / driving gear)
    double COUNTS_PER_INCH = (NEVEREST_40_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    double DRIVE_SPEED = 0.5;
    double TURN_SPEED = 0.2;


}
