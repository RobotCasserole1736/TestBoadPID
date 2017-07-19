package org.usfirst.frc.team1736.robot;

import org.usfirst.frc.team1736.lib.Calibration.CalWrangler;
import org.usfirst.frc.team1736.lib.Calibration.Calibration;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebPlots;
import org.usfirst.frc.team1736.lib.WebServer.CasseroleWebServer;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Utility;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	CasseroleWebServer webServer;
	
	//Motor Controller for driving the mass
	private CANTalon motorCTRL;
	
	//power distribution panel
	private PowerDistributionPanel pdp;
	
	//Controls what sort of closed-loop control we will use
	// 0 = Open-loop        (Amp in %vbus)
	// 1 = Velocity Control (Amp in RPM)
	// 2 = Position Control (Amp in deg)
	// 3 = Current Control  (Amp in A)
	Calibration ctrlMode;
	
	//Defines the sort of test desired input to the system
	// 0 = No input
	// 1 = Step
	// 2 = Sine wave
	Calibration cycleType;
	//Defines the length of the test cycle in seconds
	Calibration cycleLength;
	
	//Defines the percentage of time to stay "on" during a step test cycle
	Calibration cycleStepOnPct;
	
	//Maximum deviation-from-zero of the test cycle
	Calibration cycleAmplititude;
	//Defines the frequency of sinusoidal test waves
	Calibration cycleSineFrequency;
	
	//Gains for position mode
	Calibration posKp;
	Calibration posKi;
	Calibration posKd;

	//Gains for speed mode
	Calibration spdKp;
	Calibration spdKi;
	Calibration spdKd;
	Calibration spdKf;
	
	//Gains for Current mode
	Calibration curKp;
	Calibration curKi;
	Calibration curKd;
	Calibration curKf;
	
	
	//True if we are running a cycle, false otherwise.
	boolean cycleActive;
	boolean cycleActivePrev;
	
	//User button previous value
	boolean userButtonPrev;
	
	//Cached vvalue of the srx control mode
	double presentCtrlMode;
	
	//Cycle time calculations (s)
	double cycleElapsedTime;
	double cycleStartTime;
	

	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		webServer = new CasseroleWebServer();
		webServer.startServer();
		motorCTRL = new CANTalon(0);
		pdp = new PowerDistributionPanel();
		
		ctrlMode = new Calibration("Control Mode", 0,0,2);
		cycleType = new Calibration("Cycle Type", 0,0,2);
		cycleLength = new Calibration("Cycle Length S", 5,0,15);
		cycleAmplititude = new Calibration("Cycle Amplititude", 100,-3000,3000);
		cycleStepOnPct = new Calibration("Step Cycle On Pct", 75,0,100);
		cycleSineFrequency = new Calibration("Sine Cycle Frequency Hz", 0,0,2);
		
		posKp = new Calibration("Gain Position P", 0);
		posKi = new Calibration("Gain Position I", 0);
		posKd = new Calibration("Gain Position D", 0);
		spdKp = new Calibration("Gain Speed P", 0);
		spdKi = new Calibration("Gain Speed I", 0);
		spdKd = new Calibration("Gain Speed D", 0);
		spdKd = new Calibration("Gain Speed F", 0);
		curKp = new Calibration("Gain Current P", 0);
		curKi = new Calibration("Gain Current I", 0);
		curKd = new Calibration("Gain Current D", 0);
		curKf = new Calibration("Gain Current F", 0);

		CalWrangler.loadCalValues();
		
		CasseroleWebPlots.addNewSignal("Motor Desired", "varies");
		
		CasseroleWebPlots.addNewSignal("Motor Speed", "RPM");
		CasseroleWebPlots.addNewSignal("Motor Pos", "Deg");
		
		CasseroleWebPlots.addNewSignal("Motor Voltage", "V");
		CasseroleWebPlots.addNewSignal("Motor Current", "A");
		CasseroleWebPlots.addNewSignal("PDP Voltage", "V");
		CasseroleWebPlots.addNewSignal("PDP Current", "A");
		
		
		cycleActive = false;
		cycleActivePrev = false;
		userButtonPrev = false;
		
		
		
	}
	
	
	/**
	 * This function is run when the robot enters disabled
	 */
	@Override
	public void disabledInit(){
		shutOffMotor();
		cycleElapsedTime = 0;
	}
	
	
	/**
	 * This function is called periodically during disabled mode
	 */
	@Override
	public void disabledPeriodic(){
		
	}

	/**
	 * This function is run when the robot enters Autonomous
	 */
	@Override
	public void autonomousInit() {
		shutOffMotor();
		//Do Nothing
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		//Do Nothing
	}
	
	/**
	 * This function is run when the robot enters Teleop
	 */
	@Override
	public void teleopInit() {
		//For safety, ensure we start at motor off
		shutOffMotor();
	}
     

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		double motorDesired = 0;

		//Process user button to find rising edges
		boolean button_rising_edge = false;
		boolean cur_button_state = Utility.getUserButton();
		if(cur_button_state == true & userButtonPrev == false){
			button_rising_edge = true;
		}
		
		
		//Rising edges indicate a cycle should be started
		// Or the present cycle should be stopped.
		if(button_rising_edge == true){
			//switch cycle state
			cycleActive = !cycleActive;
		}
		
		//Handle transition from cycle-active to cycle-not-active
		if(cycleActive == false && cycleActivePrev == true){
			//When ending a cycle, ensure the motor is stopped.
			shutOffMotor(); 
			cycleElapsedTime = 0;
		}
		
		//Handle transition from cycle-not-active to cycle-active
		if(cycleActive == true && cycleActivePrev == false){
			
			//Assign the control mode to the talon
			presentCtrlMode = ctrlMode.get();
			if(presentCtrlMode== 0){
				motorCTRL.changeControlMode(TalonControlMode.PercentVbus);
			} else if (presentCtrlMode == 1){
				motorCTRL.changeControlMode(TalonControlMode.Speed);
			} else if (presentCtrlMode == 2){
				motorCTRL.changeControlMode(TalonControlMode.Position);
			} else {
				motorCTRL.changeControlMode(TalonControlMode.Disabled);
			}
			ctrlMode.acknowledgeValUpdate();
			
			cycleStartTime = Timer.getFPGATimestamp();
			
		}

		
		if(cycleActive == true){
			//We are presently running a cycle
			
			//Get cycle elapsed time
			cycleElapsedTime = Timer.getFPGATimestamp() - cycleStartTime;
			
			if(cycleType.get() == 1){
				if(cycleElapsedTime > cycleLength.get() * cycleStepOnPct.get() / 100.0 ){
					motorDesired = 0;
				} else {
					motorDesired = cycleAmplititude.get();
				}
			} else if(cycleType.get() == 2) {
				//tbd - sine wave
			} else {
				shutOffMotor();
			}
			
			//check for end of cycle
			if(cycleElapsedTime > cycleLength.get() ){
				cycleActive = false;
			}
			
			
			//Update data plots
			double timeNow = Timer.getFPGATimestamp();
			CasseroleWebPlots.addSample("Motor Desired", timeNow, motorDesired);
			CasseroleWebPlots.addSample("Motor Speed",  timeNow, motorCTRL.getSpeed());
			CasseroleWebPlots.addSample("Motor Pos",   timeNow, motorCTRL.getPosition());
			CasseroleWebPlots.addSample("Motor Voltage",timeNow,motorCTRL.getOutputVoltage());
			CasseroleWebPlots.addSample("Motor Current",timeNow,motorCTRL.getOutputCurrent());
			CasseroleWebPlots.addSample("PDP Voltage",timeNow, pdp.getVoltage());
			CasseroleWebPlots.addSample("PDP Current",timeNow, pdp.getTotalCurrent());
			
			
			
			
		} else {
			//No cycle, don't run anything
			shutOffMotor();
			
			//Update calibration values
			updateCalValues();
			
		}
			
			
			
	    //Update prev state values
		userButtonPrev = cur_button_state;
		cycleActivePrev = cycleActive;
	
	}
	
	/** 
	 * Forces motor command to zero
	 */
	private void shutOffMotor(){
		motorCTRL.changeControlMode(TalonControlMode.PercentVbus);
		motorCTRL.set(0);
	}
	
	
	private void updateCalValues(){
		if(presentCtrlMode == 1){
			motorCTRL.setP(spdKp.get());
			motorCTRL.setI(spdKi.get());
			motorCTRL.setD(spdKd.get());
			motorCTRL.setF(spdKf.get());
		} else if (presentCtrlMode == 2) {
			motorCTRL.setP(posKp.get());
			motorCTRL.setI(posKi.get());
			motorCTRL.setD(posKd.get());
			motorCTRL.setF(0);
		} else if (presentCtrlMode == 3){
			motorCTRL.setP(curKp.get());
			motorCTRL.setI(curKi.get());
			motorCTRL.setD(curKd.get());
			motorCTRL.setF(curKf.get());
		} else {
			motorCTRL.setP(0);
			motorCTRL.setI(0);
			motorCTRL.setD(0);
			motorCTRL.setF(0);
		}
	}

}

