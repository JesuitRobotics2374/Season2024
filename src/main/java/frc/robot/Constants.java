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

    // MANIPUlATIR Subsystem
    public static final int SHOOTER_MOTOR_A_ID = 32;
    public static final int SHOOTER_MOTOR_B_ID = 31;
    public static final int INTAKE_MOTOR_ID = 33;
    public static final int SENSOR_PORT = 18;

    // ANGLE Calculator
    public static final double armLength = 0.62;
    public static final double armAngleOffset = Math.toRadians(102); // TODO
    public static final double deltaHeight = 2 - 0.28;
    public static final double launchVelocity = 11.8;
    public static final double FEED_FORWARD_VOLTAGE = 0.3;
    public static final int LEFT_CLIMBER_MOTOR_ID = 8;
    public static final int RIGHT_CLIMBER_MOTOR_ID = 9;
}
