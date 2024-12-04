// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import static frc.robot.settings.Constants.PS4Driver.*;

import frc.robot.commands.ClimberCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final boolean climberExists = Preferences.getBoolean("climber", true);
 
  private DrivetrainSubsystem driveTrain;
  private Drive defaultDriveCommand;
  private PS4Controller driverController;
  private PS4Controller codriverController;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;
  private Climber climber;

  BooleanSupplier ZeroGyroSup;
  BooleanSupplier AmpAngleSup;
  BooleanSupplier falseSup;
  DoubleSupplier zeroSup;
  BooleanSupplier climberDown;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Preferences.initBoolean("Climber", true);

    DataLogManager.start(); //Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); //Joystick Data logging

    driverController = new PS4Controller(DRIVE_CONTROLLER_ID);
    codriverController = new PS4Controller(CODRIVERCONTROLLER);
    PDP = new PowerDistribution(1, ModuleType.kRev);
    ZeroGyroSup = driverController::getPSButton;
    AmpAngleSup = driverController::getSquareButton;
    climberDown = codriverController::getPSButton;
    zeroSup = ()->0;
    falseSup = ()->false;
    
    driveTrainInst();
     if(climberExists) {climberInst();}
    configureBindings();
    // Configure the trigger bindings
   
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();
    defaultDriveCommand = new Drive(
      driveTrain, 
      () -> false,
      () -> modifyAxis(-driverController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driverController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
      driveTrain.setDefaultCommand(defaultDriveCommand);

  }
  private void climberInst() {
   climber = new Climber();
   climber.setDefaultCommand(new ClimberCommand(
    climber,
    ()->modifyAxis(codriverController.getRightY(), DEADBAND_NORMAL),
    ()->modifyAxis(codriverController.getLeftY(), DEADBAND_NORMAL),
    climberDown));



   }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(AmpAngleSup).onTrue(new InstantCommand(driveTrain::pointWheelsInward, driveTrain));
    SmartDashboard.putData("drivetrain", driveTrain);
    Command setGyroTo180 = new InstantCommand(()->driveTrain.zeroGyroscope(180)) {
      public boolean runsWhenDisabled() {
              return true;
      };
    };
    SmartDashboard.putData("set gyro 180", setGyroTo180);
    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));

    InstantCommand setOffsets = new InstantCommand(driveTrain::setEncoderOffsets) {
      public boolean runsWhenDisabled() {
        return true;
      };
    };
    SmartDashboard.putData("set offsets", setOffsets);

/* bindings:
 *    L2: aim at speaker and rev up shooter to max (hold)
 *    L1: manually feed shooter (hold)
 *    R2: shoot if everything is lined up (hold)
 *    Circle: lineup with the amp +shoot at amp speed (hold)
 *    D-Pad down: move shooter up manually (hold)
 *    R1: aim shooter at amp (hold)
 *    Options button: collect note from human player
 *    Square: auto-pick up note
 *    Touchpad: manually turn on Intake (hold) [only works if intake code doesn't exist in IndexCommand]
 *    L1,L2,R1,R2 held: aim shooter at speaker and set shooter to shooter speed
 * 
 *  operator:
 *    Triangle: climber up (hold)
 *    Cross: climber down (hold)
 *    R1: auto pickup note from ground (hold)
 *    
 */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0)+1);
  }
  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  public void teleopInit() {
  }

  public void teleopPeriodic() {
    SmartDashboard.putData(driveTrain.getCurrentCommand());
  }
 
  public void logPower(){
    for(int i = 0; i < 16; i++) { 
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }
  public void robotPeriodic() {
    // logPower();
  }
  public void disabledPeriodic() {
  }

  public void disabledInit() {
  }
}
