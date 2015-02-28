import java.net.MalformedURLException;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;

import lejos.remote.ev3.RMIRegulatedMotor;
import lejos.remote.ev3.RemoteEV3;

public class Robot {
	
    	public static final String USB_ADDR = "10.0.1.1";
    	public static final int MOTOR_SPEED_DEG_PER_SECONDS = 100;
    	protected RemoteEV3 robot;
    	protected RMIRegulatedMotor motorA;
    	protected RMIRegulatedMotor motorB;
    	
    	public Robot() throws RemoteException, MalformedURLException, NotBoundException {

    		System.out.println("Connecting to EV3...");
    		robot = new RemoteEV3(USB_ADDR);    		
    		System.out.println("EV3 connected.");

    		robot.getAudio().systemSound(2);
    		System.out.println("Connecting motors...");
    		try{
    			motorA = robot.createRegulatedMotor("A", 'L');
    			motorA.setSpeed(MOTOR_SPEED_DEG_PER_SECONDS);
    			motorA.flt(true);
    		}catch(Exception e){
    			System.err.println("Couldn't connect motor A: " + e.getMessage());
    		}

    		try{
    			motorB = robot.createRegulatedMotor("B", 'L');
    			motorB.setSpeed(MOTOR_SPEED_DEG_PER_SECONDS);
    			motorB.flt(true);
    		}catch(Exception e){
    			System.err.println("Couldn't connect motor B: " + e.getMessage());
    		}
    		
    		System.out.println("Motors connected.");
    		System.out.println("EV3 setup finished.");
    	}

    	public void rotateMotorA(int angle){
    		try{
    			motorA.rotate(angle);
    		} catch (RemoteException e){
    			System.err.println("Unable to rotate motor A: " + e.getMessage());
    		}
    	}

    	public void rotateMotorB(int angle){
    		try{
    			motorB.rotate(angle);
    		} catch (RemoteException e){
    			System.err.println("Unable to rotate motor B: " + e.getMessage());
    		}
    	}
    	
    	public void rotateToMotorA(int angle){
    		try{
    			motorA.rotateTo(angle);
    		} catch (RemoteException e){
    			System.err.println("Unable to rotate motor A: " + e.getMessage());
    		}
    	}

    	public void rotateToMotorB(int angle){
    		try{
    			motorB.rotateTo(angle);
    		} catch (RemoteException e){
    			System.err.println("Unable to rotate motor B: " + e.getMessage());
    		}
    	}

    	public void close(){
    		try{
    			motorA.close();
    		} catch (RemoteException e){
    			System.err.println("Unable to close motor A: " + e.getMessage());
    		}

    		try{
    			motorB.close();
    		}catch (RemoteException e){
    			System.err.println("Unable to close motor B: " + e.getMessage());
    		}

    		robot.getAudio().systemSound(3);
    		System.out.println("Remote EV3 connection closed.");
    	}
}