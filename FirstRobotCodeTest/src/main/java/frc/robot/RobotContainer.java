package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final static XboxController driver = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    // private final ArmUpdated arm = new ArmUpdated(driver); 
    // private final Intake intake = new Intake(); //set default command when its made
    // private final Climber climber = new Climber();

    // // Commands
    // private final ArmAngle angleCommand = new ArmAngle(arm);
    // private final InNOut in = new InNOut(intake);

    // private final SlewRateLimiter filter = new SlewRateLimiter(0.5);
    // private final SlewRateLimiter driveFilter = new SlewRateLimiter(3);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -0.25*driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        //arm.setDefaultCommand(angleCommand);
        //intake.setDefaultCommand(in);
        //camera
        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // new Trigger(()->driver.getAButtonPressed()).onTrue(Commands.runOnce(
        //     ()->arm.setArmToAmp(), arm));
        // new Trigger(()->driver.getRightBumperPressed()).onTrue(Commands.runOnce(
        //     ()->arm.setArmToIntake(), arm));
        // new Trigger(()->driver.getRightBumperReleased() && driver.getYButtonReleased()  && driver.getXButtonReleased() && driver.getAButtonReleased()).onTrue(Commands.runOnce(
        //     ()->arm.setArmTo45(), arm));
        // new Trigger(()->driver.getYButtonPressed()).onTrue(Commands.runOnce(
        //     ()->arm.setArmToLimelightTrack(), arm));
        // new Trigger(()->driver.getXButtonPressed()).onTrue(Commands.runOnce(
        //     ()->arm.setArmToClimb(), arm));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new SequentialCommandGroup(
        // new AutoAim(arm), 
        // new AutoShoot(intake), 
        // new AutoDown(arm),
        // new ParallelCommandGroup(new exampleAuto(s_Swerve), new IntakeOn(intake)), 
        // //new DriveBack(s_Swerve),
        // new AutoAim(arm), 
        // new AutoShoot(intake));
    }

