package frc.robot;

public class Constants {
    public static final String CAN_BUS_NAME_CANIVORE = "FastFD";
    public static final String CAN_BUS_NAME_ROBORIO = "rio";
    public static final String CAN_BUS_NAME_DRIVETRAIN = CAN_BUS_NAME_CANIVORE;

    // Contoller IDs
    public static final int CONTROLLER_USB_PORT_DRIVER = 0; // Drivers Controller
    public static final int CONTROLLER_USB_PORT_OPERATOR = 1; // Ordanence operators controller

    public static final String DRIVER_READOUT_TAB_NAME = "Driver Readout";

    // CHASSIS Subsystem
    public static final String TEST_ROBORIO_SERIAL_NUMBER = "0316b2d6"; // serial number of Swervee roborio

    // ARM Subsystem
    public static final int RIGHT_ARM_MOTOR_ID = 6;
    public static final int LEFT_ARM_MOTOR_ID = 7;
    public static final int ARM_ENCODER_ID = 25;
    public static final double FORWARD_SOFT_STOP = 0.15;
    public static final double BACKWARD_SOFT_STOP = -0.265;

    // PID
    public static final double ARM_PID_P = 4.0;
    public static final double ARM_PID_I = 0.25;
    public static final double ARM_PID_D = 0.0;

    // Profiled PID
    public static final double P_ARM_PID_P = 2.0;
    public static final double P_ARM_PID_I = 0.05;
    public static final double P_ARM_PID_D = 0.1;

    // Center of field
    public static final double CENTER_FIELD_X = 8.4;

    // Translation2d
    public static final double RED_SPEAKER_X = 16.54;
    public static final double RED_SPEAKER_Y = 5.55;

    public static final double BLUE_SPEAKER_X = 0;
    public static final double BLUE_SPEAKER_Y = 5.55;

    // Align speeds
    public static final double ALIGN_SPEED_THRESHOLD = 0.1;
    public static final double ALIGN_SPEED_FAST = 1.2;
    public static final double ALIGN_SPEED_SLOW = 0.8;

    // MANIPUlATIR Subsystem
    public static final int SHOOTER_MOTOR_A_ID = 32;
    public static final int SHOOTER_MOTOR_B_ID = 31;
    public static final int INTAKE_MOTOR_ID = 33;
    public static final int SENSOR_PORT = 18;

    // ANGLE Calculator
    public static final double armLength = 0.62;
    public static final double armAngleOffset = Math.toRadians(121.5);
    public static final double deltaHeight = 2 - 0.30;
    public static final double launchVelocity = 14.2;
    public static final double dragCoefficient = 0.0; // Positive - TODO
    public static final double FEED_FORWARD_VOLTAGE = 0.3;

    // CLIMBER Subsystem
    public static final int LEFT_CLIMBER_MOTOR_ID = 8;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 9;
}
