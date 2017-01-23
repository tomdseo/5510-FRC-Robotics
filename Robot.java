package org.usfirst.frc.team5510.robot;

import java.sql.Time;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Robot extends SampleRobot implements PIDOutput {
    RobotDrive myRobot;
    Joystick stick;
    Joystick joystick;
    
    AHRS ahrs;
    
    //limitswitch
    DigitalInput limitSwitch;
    
    //collision detection
    double last_accel_x;
    double last_accel_y;
    final static double kCollisionThreshold = 1f;
    double currentTime;
    
    double driveSpeedMod;
    
    //Auto chooser
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    final String posOne = "Position One";
    final String posTwo = "Position Two";
    final String posThree = "Position Three";
    final String posFour = "Position Four";
    final String posFive = "Position Five";
    
    final String[] obstacles = {
    		"lowBar",
    		"port",
    		"cheval",
    		"moat",
    		"rampart",
    		"rockWall",
    		"roughTerrain"
    };
    
    SendableChooser chooser;
    SendableChooser obstacleChooser;
    
    //max speed setter
    final String hundredPercent = "100%";
    final String ninetyPercent = "90%";
    SendableChooser chooser1;
    
    //mechanisms
    Talon arm1;
    Talon arm2;
    Spark intakeRight;
    Spark intakeLeft;
    double speed;
    
    //direction switch
    boolean down = false;
    boolean forward = true;
    
    //auto drive straight
    PIDController turnController; 
    double rotateToAngleRate;
    static double kP = 0.001; //increase until oscillations and then half
    static double kI = 0.00; //increase until offset is corrected in time
    static double kD = 0.00; //increase until quick enough
    static final double kF = 0.00; 
    static final double kToleranceDegrees = 2.0f;
    
    //intake control
    boolean inFoDown = false;
    boolean inFo = false;
    boolean inBaDown = false;
    boolean inBa = false;
    
    //intake speed mods
    double speedMod = 0.8;
    boolean rightMod = false;
    boolean leftMod = false; 
    
    //Servo
    Servo servo;
    
    Victor right1;
    Victor right2;
    Victor left1;
    Victor left2;

    //auto values
    boolean ranOnce = false;
    Timer t;
    boolean isBumped = false;
    long currentAutoTime;
    
    
    public Robot() {
    	limitSwitch = new DigitalInput(9);
    	t = new Timer();
        stick = new Joystick(0);     
        joystick = new Joystick(1); //use joystick for testing PID
        right1 = new Victor(1);
        right2 = new Victor(2);
        left1 = new Victor(3);
        left2 = new Victor(4);
        myRobot = new RobotDrive(left1, left2, right1, right2);
        
        //SWITCH THESE IF IT RUNS BACKWARDS
        arm1 = new Talon(8); //arm
        arm2 = new Talon(6);
        intakeRight = new Spark(0);
        intakeLeft = new Spark(5);
        
        //SET SPEED OF ARMS HERE
        speed = 0.7;
        
        //Servo
        servo = new Servo(7);
        
        try{
        	ahrs = new AHRS(SerialPort.Port.kMXP); 
        } catch(RuntimeException e){
        	DriverStation.reportError("navx broken: " + e.getMessage(), true);
        }
        
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this); 
        turnController.setInputRange(-180.0f,  180.0f); 
        turnController.setOutputRange(-1.0, 1.0); 
        turnController.setAbsoluteTolerance(kToleranceDegrees); 
        turnController.setContinuous(true); 

        LiveWindow.addActuator("DriveSystem", "RotateController", turnController); 

    }
    
    @Override
	public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto modes", chooser);
        
        obstacleChooser = new SendableChooser();
        for(String obs: obstacles){
        	obstacleChooser.addObject(obs, obs);
        }
        SmartDashboard.putData("Obstacle Type", obstacleChooser);
        
        chooser1 = new SendableChooser();
        chooser1.addDefault("100%", hundredPercent);
        chooser1.addObject("90%", ninetyPercent);
        SmartDashboard.putData("Battery Percent", chooser1);
    }
    
    
    boolean hitWall = false;
	/*public void autonomous() {
    	String autoSelected = (String) chooser.getSelected();
    	String obstacleSelected = (String) obstacleChooser.getSelected();
    	
    	if(isAutonomous() && !isEnabled()){
    		ranOnce = false;
    		hitWall = false;
    	}
    	
    	while(isAutonomous() && isEnabled()){
    		if(!ranOnce){
				currentAutoTime = System.currentTimeMillis();
				ranOnce = true;
			}
    		
    		//autoObstacle(obstacleSelected);
    		if(isBefore(1000)){ //first second do this
				autoDrive(-0.5);
			}
    		else{
    			autoDrive(0);
    		}
    		/*
    		switch(autoSelected) {
    		case posOne:
    			if(!isBumped && !hitWall){
    				autoDrive(-0.25);
    				hitWall = true;
    			}
    			if(hitWall){
    				myRobot.drive(0.25, 0);
    				Timer.delay(0.5);
    				
    			}
    			break;
    		case posTwo:
    			break;
    		case posThree:
    			break;
    		case posFour:
    			break;
    		case posFive:
    			break;
    		
    		
    		default:
            
    			if(isBefore(1000)){ //first second do this
    				autoDrive(-0.5);
    			}
    			else if(isBefore(4000)){
    				autoDrive(0);
    			}
    			else if(isBefore(6000) && !isBumped){
    				collisionDetection();
    				autoDrive(-0.5);
    			}
    			else if(isBefore(8000)  && isBumped){
    				myRobot.drive(.5, 0);
    				Timer.delay(0.5);
    				myRobot.drive(0, 0);
    				Timer.delay(0.3);
    				autoDrive(0.5, 90);
    			}
    			else if(isBefore(10000)){
    				autoIntakeToggle(true);
    				Timer.delay(0.5);
    				autoServoToggle(true);
    				Timer.delay(1);
    				autoIntakeToggle(false);
    				autoServoToggle(false);
    			}
    			else{
    				autoDrive(0);
    			}
    			break;
    		}*/
    	//} 
    //}
    
public void autonomous() {
    /*	
	myRobot.setSafetyEnabled(false);
	servo.set(0.8);//servo to down pos
	setArms(-0.4);//arms down fast
	Timer.delay(1);
	setArms(-0.3);//arms down slow
	Timer.delay(0.5);
	setArms(0);//stop arms
	Timer.delay(1);
	
    myRobot.drive(0.5, 0.0);//back for 4 sec	
    Timer.delay(4);		
    myRobot.drive(0.0, 0.0);//stop robot
    Timer.delay(0.75);
	
    //reset value
    servo.set(0.8);
    myRobot.drive(0, 0);
    setIntake(0);
    */
    }
    
    private void autoObstacle(String key){
    	if(key.equals(obstacles[0])){ //low bar
			autoDrive(-0.5);
			Timer.delay(2);
		}
		else if(key.equals(obstacles[1])){ //port
			autoDrive(0.25);
			Timer.delay(0.75);
		}
		else if(key.equals(obstacles[2])){ //chev
			autoDrive(0.25);
			Timer.delay(0.75);
		}
		else if(key.equals(obstacles[3])){ //moat
			autoDrive(-0.75);
			Timer.delay(1.5);
		}
		else if(key.equals(obstacles[4])){ //ramparts
			autoDrive(-0.75);
			Timer.delay(1.5);
		}
		else if(key.equals(obstacles[5])){ //rock wall
			autoDrive(-.5);
			Timer.delay(2);
		}
		else if(key.equals(obstacles[6])){ //Rough Terrain
			autoDrive(-0.5);
			Timer.delay(2);
		}
    }
    
    private boolean isBefore(long time){
    	if(currentAutoTime + time < System.currentTimeMillis()){
    		return true;
    	}
    	else{
    		return false;
    	}
    }
    
    private void autoIntakeToggle(boolean motorOn){
    	if(motorOn){
    		intakeHelper(1, 1);
    	}
    	else{
    		intakeHelper(0, 0);
    	}
    }
    
    private void autoServoToggle(boolean servoOn){
    	if(servoOn){
    		servo.set(1);
    	}
    	else{
    		servo.set(0);
   
    	}
    }
    
    private void autoDrive(double power, double angle){
    	turnControllerHelper(angle);
    	myRobot.drive(power, rotateToAngleRate);
    }
    
    private void autoDrive(double power){
    	autoDrive(power, 0);
    }
    
    private void turnControllerHelper(double angle){
    	if(ahrs.getYaw() > (angle + 0.5) || ahrs.getYaw() < (angle -0.5)) {
        	turnController.setSetpoint(angle);
        	turnController.enable();
        }
    }
    
   @Override
   public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
        	curveBall();
        	servoControl();
        	collisionDetection();
        	switchDirection();
        	xboxDrive();
        	moveArms();
            intake();
            smartBoardInfo();
            smartBoardCap();
            Timer.delay(0.005);
        }
   }
   
   private void smartBoardCap(){
	   String batterySelected = (String) chooser1.getSelected();
	   if(batterySelected.equals(hundredPercent)){
		   driveSpeedMod = 1;
	   }
	   else{
		   driveSpeedMod = 0.9;
	   }
   }

   boolean slowUpOn = false;
   boolean slowUpDown = false;
   
   private void curveBall(){
	   if(stick.getPOV() == 270){
		   leftMod = true;
		   rightMod = false;
	   }
	   else if(stick.getPOV() == 90){
		   leftMod = false;
		   rightMod = true;
	   }
	   else if(stick.getRawButton(7)){
		   rightMod = false;
		   leftMod = false;
	   }
   }
   
   private void servoControl(){
	   if(stick.getRawButton(5) || stick.getRawButton(6)){
		   servo.set(0.5);
	   }
	   else{
		   servo.set(0.8);
	   }
   }
   
   private void switchDirection(){
	   if(stick.getRawButton(3)){
		   if(!down){
			   forward = !forward;
			   down = true;
		   }
	   }
	   else{
		   down = false;
	   }
   }
    
    private void xboxDrive(){
    	if(forward)
    		myRobot.tankDrive(-stick.getRawAxis(1) * 0.9, -stick.getRawAxis(5) * 0.9, true);
    	else
    		myRobot.tankDrive(stick.getRawAxis(5) * 0.9, stick.getRawAxis(1) * 0.9, true);
    }
    
    long shootTurnedOnTime;
    private void intake(){
    	
    	if(stick.getRawButton(1)){
    		if(!inFoDown && !inFo){
    			setIntake(.6);
    			shootTurnedOnTime = System.currentTimeMillis();
    			inFoDown = true;
    			inFo = true;
    			inBa = false;
    		}
    		if(!inFoDown && inFo){
    			inFo = false;
    			setIntake(0);
    			inFoDown = true;
    		}
        }
    	else{
    		inFoDown = false;
    	}
    	
    	
    	if(stick.getRawButton(2)){
    		if(!inBaDown && !inBa){
    			setIntake(-0.9);
    			shootTurnedOnTime = System.currentTimeMillis();
    			inBaDown = true;
    			inBa = true;
    			inFo = false;
    		}
    		if(!inBaDown && inBa){
    			inBa = false;
    			setIntake(0);
    			inBaDown = true;
    		}
        }
    	else{
    		inBaDown = false;
    	}
    	
    	if(stick.getRawButton(4)){
    		setIntake(0);
    		inBa = false;
    		inFo = false;
    	}
    	if(shootTurnedOnTime + 5000 <  System.currentTimeMillis()){
    		setIntake(0);
    		inBa = false;
    		inFo = false;
    	}
    }
    
    private void setIntake(double power){
    	if(rightMod){
    		intakeHelper(power*speedMod, power);
    	}
    	else if(leftMod){
    		intakeHelper(power, power*speedMod);
    	}
    	else{
    		intakeHelper(power, power);
    	}
    }
    
    private void intakeHelper(double right, double left){
    	intakeRight.set(-right);
		intakeLeft.set(left);
    }
    
    private void moveArms(){
    	if(stick.getRawAxis(3) > 0.05){
    		setArms(speed * stick.getRawAxis(3));
    	}
    	else if(stick.getRawAxis(2) > 0.05){
    		setArms(-speed * stick.getRawAxis(2));
    	}
 	   else{
 		   setArms(0);
 	   }
    }
    
    private void setArms(double power){
    	arm1.set(-power);
		arm2.set(power);
    }
    
    private void collisionDetection(){
    	boolean collisionDetected = false; 
    	
    	double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX(); 
    	double currentJerkX = curr_world_linear_accel_x - last_accel_x; 
    	last_accel_x = curr_world_linear_accel_x; 
    	double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY(); 
    	double currentJerkY = curr_world_linear_accel_y - last_accel_y; 
    	last_accel_y = curr_world_linear_accel_y; 
    	
    	if ( ( Math.abs(currentJerkX) > kCollisionThreshold ) || 
    	     ( Math.abs(currentJerkY) > kCollisionThreshold) ) { 
    	    collisionDetected = true; 
    	} 
    	SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
    	
    	//Rumble that fucker
    	if(collisionDetected && !isAutonomous()){
    		currentTime = System.currentTimeMillis();
    		stick.setRumble(RumbleType.kLeftRumble, 1);
			stick.setRumble(RumbleType.kRightRumble, 1);
			isBumped = true;
			
    	}
    	else if(currentTime+250 < System.currentTimeMillis()){
    		stick.setRumble(RumbleType.kLeftRumble, 0);
			stick.setRumble(RumbleType.kRightRumble, 0);
    	}
    }
   
    public void smartBoardInfo(){

    	 SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected()); 
    	 SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating()); 
    	 SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw()); 
    	 SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch()); 
    	 SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());  
    	 SmartDashboard.putNumber(   "IMU_YawRateDPS",       ahrs.getRate()); 
    	 SmartDashboard.putNumber(   "IMU_Accel_X",          ahrs.getWorldLinearAccelX()); 
    	 SmartDashboard.putNumber(   "IMU_Accel_Y",          ahrs.getWorldLinearAccelY()); 
    	 SmartDashboard.putBoolean(  "IMU_IsMoving",         ahrs.isMoving()); 
    	 SmartDashboard.putBoolean(  "IMU_IsRotating",       ahrs.isRotating());            
    	 SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX()); 
    	 SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY()); 
    	 SmartDashboard.putBoolean(  "isGoingForward",       forward);
   }
    
	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}   
	
	boolean threeDown = false;
	boolean fourDown = false;
	public void test(){
		//testOne();
	}
	
	private void testOne(){
		if(joystick.getRawButton(1)){
			autoDrive(0.75);
		}
		if(joystick.getRawButton(3)){
			if(!threeDown){
				threeDown = true;
				kP += 0.001;
			}
		}
		else {
			threeDown = false;
		}
		if(joystick.getRawButton(4)){
			if(!fourDown){
				fourDown = true;
				kD += 0.001;
			}
		} 
		else{
			fourDown = false;
		}
		SmartDashboard.putNumber("kP Value: ", kP);
		SmartDashboard.putNumber("kD Value: ", kD);
	}
}