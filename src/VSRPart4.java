import lejos.utility.Matrix;


public class VSRPart4 {

	private TrackerReader tracker;
	private Robot robot;
	
	private double minError = 20;
	private double gain = 0.2; //0.1-0.5
	private double alpha = 0.2;//0.1-0.5
	private int deltaAngle = 20;
	private double error = Double.MAX_VALUE;
	
	private Matrix mdf = null;
	
	public static void main(String [] args)
	{
		VSRPart4 vsr = new VSRPart4();
		try
		{
			vsr.calculateJacobian(0,0);
			vsr.inverseNewton();
		}
		catch(Exception e){e.printStackTrace();}
		vsr.disconnect();
	}
	
	
	public VSRPart4(){
		try{ robot = new Robot();
		}catch(Exception e){
			System.err.println("Couldn't connect to the EV3: " + e.getMessage());
			return;
		}
		tracker = new TrackerReader();
		tracker.launchService(); //locking, until it's tracking an object
	}
	
	public void disconnect(){
		robot.rotateToMotorA(0);
		robot.rotateToMotorB(0);
		robot.close();
		tracker.close();
	}
	
	
	public void calculateJacobian(double a1, double a2){
		System.err.println("Updating Jacobian");
		double[] cor = forwarKin(a1,a2);
		double[] cor_1 = forwarKin(a1+deltaAngle,a2);
		double[] cor_2 = forwarKin(a1,a2+deltaAngle);
		double[][] df = new double[][]
				{
				{cor_1[0]-cor[0],cor_2[0]-cor[0]},
				{cor_1[1]-cor[1],cor_2[1]-cor[1]},
				};
		mdf = new Matrix(df);
		robot.rotateToMotorA((int)a1);
		robot.rotateToMotorB((int)a2);
		System.out.println("["+mdf.get(0, 0)+" "+mdf.get(0, 1));
		System.out.println(mdf.get(1, 0)+" "+mdf.get(1, 1)+"]");
	}
	
	public void inverseNewton() throws InterruptedException{
		double a1,a2;
		
		double[][] f= new double[][]{{tracker.x},{tracker.y}};
		double[][] dx = new double[][]{{0},{0}};
		double[][] dy = new double[][]{{0},{0}};
		double[][] y = new double[][]{{tracker.targetx},{tracker.targety}};
		Matrix mdx = new Matrix(dx); //dQ
		Matrix mdy = new Matrix(dy); //dS
		Matrix my = new Matrix(y);
		error = Double.MAX_VALUE;
		
		while(error>minError){
			double dist = Math.hypot(tracker.x-tracker.targetx, tracker.y-tracker.targety);
			gain = 1.-(dist/500);
			if(gain>1)
				gain=1;
			else if(gain<0.1)
				gain=0.1;
			
			y = new double[][]{{tracker.targetx},{tracker.targety}};
			my = new Matrix(y);
			
			double oldX=f[0][0];
			double oldY=f[1][0];
			
			Matrix mf = new Matrix(f);
			Matrix minv = mdf.inverse();
			Matrix merr = my.minus(mf); //this is error
			Matrix minverr = minv.times(merr);
			mdx = minverr.times(gain);
			a1=mdx.get(0, 0);
			a2=mdx.get(1, 0);
			
			double angle = Math.hypot(a1,a2);
			if(angle>10){
				mdx = minverr.times(10/angle);
				a1=mdx.get(0, 0);
				a2=mdx.get(1, 0);
			}
				
			robot.rotateMotorA((int)a1);
			robot.rotateMotorB((int)a2);
			Thread.sleep(100);
			f = new double[][]{{tracker.x},{tracker.y}};
			dy = new double[][]{{f[0][0]-oldX},{f[1][0]-oldY}};
			mdy = new Matrix(dy);
			
			Matrix mJacTimedQ=mdf.times(mdx);
			Matrix mdS_Minus_JdQ=mdy.minus(mJacTimedQ);
			Matrix mTransDX = mdx.transpose();
			Matrix mNumerator = mdS_Minus_JdQ.times(mTransDX);
			Matrix mDenominator = mTransDX.times(mdx);
			Matrix mDivision = mNumerator.times(1/mDenominator.get(0, 0));
			Matrix mdJac = mDivision.times(alpha);
			mdf.plusEquals(mdJac); //here update the jacobian....
			
			error = Math.hypot(tracker.x-tracker.targetx, tracker.y-tracker.targety);
			System.out.println("Error "+ error);
			System.out.println("["+mdf.get(0, 0)+" "+mdf.get(0, 1));
			System.out.println(mdf.get(1, 0)+" "+mdf.get(1, 1)+"]");
			if(!tracker.isConnected) //tracker is disconnected or lost
				break;
		}
		
		if(tracker.isConnected)
			System.err.println("TARGET REACHED!!! with error: "+(int)error+" pixels, max error allowed: "+minError);
		Thread.sleep(5000);
		
	}
	
	public double[] forwarKin(double a1, double a2) {
		robot.rotateToMotorA((int)a1);
		robot.rotateToMotorB((int)a2);
		try {Thread.sleep(100);
		} catch (InterruptedException e) {}
		return new double[]{tracker.x,tracker.y};
	}
}
