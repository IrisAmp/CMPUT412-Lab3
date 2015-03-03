import lejos.utility.Matrix;


public class VisualServoRobot {

	private TrackerReader tracker;
	private Robot robot;
	
	private double maxLocalIterations = 100.;
	private double minError = 20;
	private double gain = 1;
	private int deltaAngle = 20;
	double error = Double.MAX_VALUE;
	
	private double maxLocalPoints;
	private double alpha;
	
	
	private Matrix mdf = null;
	public double x = 0;
	public double y = 0;
	public double targetX = 0;
	public double targetY = 0;
	//increase the performance...
	double[] globalErrors = null;
	
	public static void main(String [] args)
	{
		VisualServoRobot vsr = new VisualServoRobot();
		try
		{
			vsr.getInitialPosition();
			vsr.calculateJacobian(0,0);
			vsr.inverseNewtonWithInitialGuess(vsr.targetX,vsr.targetY);
			//vsr.inverseNewton(vsr.targetX,vsr.targetY,0,0);
		}
		catch(Exception e){e.printStackTrace();}
		vsr.disconnect();
	}
	
	
	public VisualServoRobot(){
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
	
	
	public void getInitialPosition(){
		System.out.println("Getting initial points");
		x=tracker.x;
		y=tracker.y;
		targetX=tracker.targetx;
		targetY=tracker.targety;
		maxLocalPoints = (int)(Math.hypot(targetX-x, targetY-y)/30) + 1;
		alpha = 1/(5*maxLocalPoints);
		System.err.println("Creating "+maxLocalPoints+" local targets");
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
	
	public void inverseNewtonWithInitialGuess(double posX,double posY){
		double[] dA = {0.,0.};
		double difX=(posX-x)/maxLocalPoints;
		double difY=(posY-y)/maxLocalPoints;
		
		for (int n=1;n<maxLocalPoints+1;n++){
			System.err.println("LOCAL POINT: "+n+" out of "+(int)maxLocalPoints);
			dA=inverseNewton(difX*n+x,difY*n+y,dA[0],dA[1], n);
			if(!tracker.isConnected) //tracker is disconnected or lost
				break;
		}
		if(tracker.isConnected)
			System.err.println("TARGET REACHED!!! with error: "+(int)error+" pixels, max error allowed: "+minError);
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	// desired position (pixels), current angles.
	public double[] inverseNewton(double posX, double posY, double a1, double a2, int iteration){
		
		double[][] f= new double[][]{{tracker.x},{tracker.y}};
		double[][] dx = new double[][]{{a1},{a2}};
		double[][] dy = new double[][]{{0},{0}};
		double[][] y = new double[][]{{posX},{posY}};
		Matrix mdx = new Matrix(dx); //dQ
		Matrix mdy = new Matrix(dy); //dS
		Matrix my = new Matrix(y);
		double er=-1;
		error = Double.MAX_VALUE;
		int n=1;
		while(error>minError){
			double oldX=f[0][0];
			double oldY=f[1][0];
			
			Matrix mf = new Matrix(f);
			Matrix minv = mdf.inverse();
			Matrix merr = my.minus(mf); //this is error
			Matrix minverr = minv.times(merr);
			mdx = minverr.times(gain);
			
			a1=mdx.get(0, 0);
			a2=mdx.get(1, 0);
			robot.rotateMotorA((int)a1);
			robot.rotateMotorB((int)a2);
			try
			{
				Thread.sleep(100);
			} catch (InterruptedException e1)
			{
				e1.printStackTrace();
			}
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
			
			error = Math.hypot(tracker.x-posX, tracker.y-posY);
			System.out.println("Error "+ error);
			System.out.println("["+mdf.get(0, 0)+" "+mdf.get(0, 1));
			System.out.println(mdf.get(1, 0)+" "+mdf.get(1, 1)+"]");
			if(n>maxLocalIterations)
				break;
			if(!tracker.isConnected) //tracker is disconnected or lost
				break;
		}
		
		a1 = mdx.get(0, 0);
		a2 = mdx.get(1, 0);
		return new double[]{a1,a2,er};
	}
	
	public double[] forwarKin(double a1, double a2) {
		robot.rotateToMotorA((int)a1);
		robot.rotateToMotorB((int)a2);
		try {Thread.sleep(100);
		} catch (InterruptedException e) {}
		return new double[]{tracker.x,tracker.y};
	}
}
