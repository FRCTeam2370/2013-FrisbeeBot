package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotTemplate extends SimpleRobot
{
   Joystick controller;
    RobotDrive driver;
    
    Jaguar motor_shooter;
    Jaguar motor_aimer;
    Jaguar motor_feeder;
    
    DigitalInput feederSwitch;
    
    DigitalInput aimerMin;
    DigitalInput aimerMax;
    
    DigitalInput ASource;
    DigitalInput BSource;

    AnalogChannel distanceSensor;
    
    Servo FeederPanel;
    
    Encoder TestEncoder;
    final double aimerMaxHeight = 39.000;
    
    public void robotInit()
    {
        controller = new Joystick(1);
        driver = new RobotDrive(9, 1, 10, 2);
        
        // the motor that controls the frisbee shooter
        motor_shooter = new Jaguar(6);
        motor_shooter.set(0.00);
        
        // the motor that controls the shooter angle
        motor_aimer = new Jaguar(7);
        motor_aimer.set(0.00);
        
        // the motor that pushes the frisbees into the shooter
        motor_feeder = new Jaguar(8);
        motor_feeder.set(0.00);
        
        // the switches that limit the aimers angle
        feederSwitch = new DigitalInput(4);
        
        aimerMin = new DigitalInput(1);
        aimerMax = new DigitalInput(2);
        
        ASource = new DigitalInput(3);
        BSource = new DigitalInput(5);
        
        distanceSensor = new AnalogChannel(6);
        
        FeederPanel = new Servo(5);
        
        TestEncoder = new Encoder(ASource, BSource);    

    }
    
    public void autonomous()
    {
        TestEncoder.setDistancePerPulse(0.006605);
        TestEncoder.reset();
        TestEncoder.start();
        motor_shooter.set(-1.00);
        
        while (isAutonomous()&& isEnabled())
        {
            
            if (TestEncoder.getDistance() < 20.000 && isAutonomous() && isEnabled())
            {
                motor_aimer.set(-0.7);
            }
            else
            {
                motor_aimer.set(0.00);
                motor_feeder.set(1.00);
            }
        }        
        
    }
    
    public void operatorControl()
    {
        TestEncoder.setDistancePerPulse(0.006605);
        TestEncoder.start();
        
        motor_shooter.set(0.00);
        motor_aimer.set(0.00);
        motor_feeder.set(0.00);
        
        while (isOperatorControl() && isEnabled())
        {
            // drive!
            if (controller.getRawButton(4))
            {
                driver.mecanumDrive_Polar(0.2, 180, 0);
            }
            else
            {
                double direction = controller.getDirectionDegrees();

                double magnitude = controller.getMagnitude();
                if (Math.abs(magnitude) < 0.1)
                {
                    magnitude = 0;
                }

                double rotation = controller.getTwist() / 2;
                if (Math.abs(rotation) < 0.1)
                {
                    rotation = 0;
                }

                driver.mecanumDrive_Polar(magnitude, direction, rotation);
            }
         
            // shooter motor control
            if (controller.getRawAxis(4) < 0)
            {
                motor_shooter.set(-1);
            }
            else
            {
                motor_shooter.set(0);
            }
            
            // aimer motor controller
            if (controller.getRawButton(11))
            {
                if((TestEncoder.getDistance() < 19.3) && isOperatorControl()&& isEnabled())
                {
                    motor_aimer.set(-0.70);
                }
                else{
                    motor_aimer.set(0.00);
                }
            }
            
            else if (controller.getRawButton(5) && (aimerMax.get() == false) && (TestEncoder.getDistance() < aimerMaxHeight))
            {
                motor_aimer.set(-0.7);
            }
            else if (controller.getRawButton(3) && (aimerMin.get() == false))
            {
                motor_aimer.set(0.30);
            }
            else
            {
                motor_aimer.set(0.00);
            }
            
            // 7 resets the encoder
            if (controller.getRawButton(7) == true)
            {
                TestEncoder.reset();
            }
            
            if (controller.getRawButton(1))
            {
                motor_feeder.set(1.00);
            }
            
            if (feederSwitch.get())
            {
                motor_feeder.set(0.00);
            }
            
            SmartDashboard.putNumber("Shooter Angle: ", TestEncoder.getDistance());
        }
    }
}
