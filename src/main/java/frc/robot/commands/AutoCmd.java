package frc.robot.commands;

import java.util.function.Supplier;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.*;  //  1/23/25 used to fix above import for phoenix6


/*    REPLACED BY ALTERNATE IMPORTS (SEE BELOW)
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
*/
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;  //Changed to Command from Command
import frc.robot.Configs.MAXSwerveModule;
// The editor is reporting errors, but the code still compiles
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoCmd extends Command {     //Changed to Command from Command

    private int configSwitch = 1;
    // 0 = move forward
    // 1 = 

    private SwerveModuleState startState = new SwerveModuleState(-.25, new Rotation2d(0));
    //private SwerveModuleState reverseState = new SwerveModuleState(-.25, new Rotation2d(0));
    //private final ShootingSubsystem shootingSubsystem;
    private final DriveSubsystem driveSubsystem;
    //private final Servo rampServo = new Servo(9);

    // public ShootAutoCmd(ShootingSubsystem shootingSubsystem)
    //     {  

    //     addRequirements(shootingSubsystem);
    // }

    public AutoCmd (DriveSubsystem driveSubsystem) {
        //this.open = open;
        //this.shootingSubsystem = shootingSubsystem;
        this.driveSubsystem = driveSubsystem;
        //addRequirements(shootingSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AutoCmd Initilizing");
        
    }

    @Override
    public void execute() {
        //System.out.println("SAutoCmd Executing");
        //ADDED TO DISPLAY VALUES OF XBOX B,Y,X BUTTONS FOR "ORIENT"" ROBOT CODE
        //System.out.println("AutoCmd Starting Shoot Speaker or Amp");
        // Rotate ramp
        //System.out.println("SAutoCmd rotating ramp with servo");
        // Shoot the note

       // driveSubsystem.m_frontLeft.setDesiredState(startState);
        //driveSubsystem.m_frontRight.setDesiredState(startState);
        //driveSubsystem.m_rearLeft.setDesiredState(startState);
       //driveSubsystem.m_rearRight.setDesiredState(startState);
        if (configSwitch >= 1) {

		    System.out.println("AutoCmd CONFIG #1 waiting 2 seconds");
		    try {
                Thread.sleep(2000); // Wait 2 sec
            } catch (InterruptedException e) {
                e.printStackTrace();
                // handle the exception...        
                // For example consider calling Thread.currentThread().interrupt(); here.
            }
		    //System.out.println("ShootAutoCmd starting loader to shoot note");
            //shootingSubsystem.mtdLoader(true,0.0); // Set Star & Wheel on
		    //System.out.println("ShootAutoCmd waiting 5 seconds");
		    try {
                Thread.sleep(2000); // Wait 2 sec
            } catch (InterruptedException e) {
                e.printStackTrace();
                // handle the exception...        
                // For example consider calling Thread.currentThread().interrupt(); here.
            }
		    //System.out.println("ShootAutoCmd Turning off Loader and Shooter motors.");
		    //shootingSubsystem.mtdShootSpeakerOrAmp(false,false); //Switches shoot to false, false to stop shooting motors.
		    //shootingSubsystem.mtdLoader(false,0.0); // Switches Star & Wheel to false to stop loading/intake.
            //System.out.println("ShootAutoCmd Moving Forward");
            //shootingSubsystem.mtdLoader(true,0.0); // Set Star & Wheel on
        }

        /* IF WE WANT TO MOVE 
        swerveSubsystem.frontLeft.setDesiredState(startState);
        swerveSubsystem.frontRight.setDesiredState(startState);
        swerveSubsystem.backLeft.setDesiredState(startState);
        swerveSubsystem.backRight.setDesiredState(startState);

        try {
            Thread.sleep(2000); // drive for 1.5 seconds
        } catch (InterruptedException e) {
                e.printStackTrace();
        }

        System.out.println("ShootAutoCmd Stopping");

        swerveSubsystem.stopModules(); */

        /*if (configSwitch >= 2) {

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // then we go backwards to reposition for shooting the note
            driveSubsystem.m_frontLeft.setDesiredState(reverseState);
            swerveSubsystem.frontRight.setDesiredState(reverseState);
            swerveSubsystem.backLeft.setDesiredState(reverseState);
            swerveSubsystem.backRight.setDesiredState(reverseState);

            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            swerveSubsystem.stopModules();

            // Now we shoot the note
            shootingSubsystem.mtdShootSpeakerOrAmp(true,false); //Switches shoot to false, false to stop shooting motors.

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            // Now turn them both off
            shootingSubsystem.mtdLoader(false, 0.0);
            shootingSubsystem.mtdShootSpeakerOrAmp(false, false);
            // then we go backwards to reposition for shooting the note
            swerveSubsystem.frontLeft.setDesiredState(startState);
            swerveSubsystem.frontRight.setDesiredState(startState);
            swerveSubsystem.backLeft.setDesiredState(startState);
            swerveSubsystem.backRight.setDesiredState(startState);

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            swerveSubsystem.stopModules();

         *///}

        /*
        -------------------------------------
        if (configSwitch >= 2) {

            System.out.println("ShootAutoCmd Moving Backwards");

            swerveSubsystem.frontLeft.setDesiredState(new SwerveModuleState(-autoSpeed, autoAngle));
            swerveSubsystem.frontRight.setDesiredState(new SwerveModuleState(-autoSpeed, autoAngle));
            swerveSubsystem.backLeft.setDesiredState(new SwerveModuleState(-autoSpeed, autoAngle));
            swerveSubsystem.backRight.setDesiredState(new SwerveModuleState(-autoSpeed, autoAngle));

            // drive for 1 second
            try {
                Thread.sleep(1000);
            } catch(InterruptedException e) {
                e.printStackTrace();
            }

            System.out.println("ShootAutoCmd Stopping");

            // Fire the note
            System.out.println("ShootAutoCmd starting loader to shoot note");
		    System.out.println("ShootAutoCmd waiting 5 seconds");
		    try {
                Thread.sleep(5000); // Wait 5 sec
            } catch (InterruptedException e) {
                e.printStackTrace();
                // handle the exception...        
                // For example consider calling Thread.currentThread().interrupt(); here.
            }
		    System.out.println("ShootAutoCmd Turning off Loader and Shooter motors.");
		    shootingSubsystem.mtdShootSpeakerOrAmp(false,false); //Switches shoot to false, false to stop shooting motors.
		    shootingSubsystem.mtdLoader(false,0.0); // Switches Star & Wheel to false to stop loading/intake.

        } */
    }

    public void performConfigZero () {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ShootAutoCmd Ending");
    }

    @Override
    public boolean isFinished() {
        System.out.println("ShootAutoCmd Finished");
        return false;
    }
}