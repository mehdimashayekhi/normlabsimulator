package es.csic.iiia.normlab.traffic.map;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.IOException;

import repast.simphony.context.Context;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridPoint;
import es.csic.iiia.normlab.traffic.TrafficSimulator;
import es.csic.iiia.normlab.traffic.agent.Car;
import es.csic.iiia.normlab.traffic.agent.Collision;
import es.csic.iiia.normlab.traffic.agent.Conflicting;
import es.csic.iiia.normlab.traffic.agent.TrafficElement;
import es.csic.iiia.normlab.traffic.car.CarAction;
import es.csic.iiia.normlab.traffic.car.CarPosition;
import es.csic.iiia.normlab.traffic.car.Pair;
import es.csic.iiia.normlab.traffic.car.context.CarContext;
import es.csic.iiia.normlab.traffic.car.context.TrafficStateCodifier;
import es.csic.iiia.normlab.traffic.factory.CarContextFactory;
import es.csic.iiia.normlab.traffic.factory.TrafficFactFactory;
import es.csic.iiia.normlab.traffic.normsynthesis.TrafficNormSynthesisSettings;
import es.csic.iiia.normlab.traffic.utils.Direction;
import es.csic.iiia.normlab.traffic.utils.Turn;
import es.csic.iiia.normlab.traffic.utils.Utilities;
import es.csic.iiia.nsm.agent.language.PredicatesDomains;
import es.csic.iiia.nsm.agent.language.SetOfPredicatesWithTerms;
import es.csic.iiia.nsm.norm.Norm;

/**
 * CarMap - Meta information management 
 * 
 * @author "Mehdi Mashayekhi"
 *
 */
public class CarMap extends TrafficMatrix {

	//---------------------------------------------------------------------------
	// Attributes 
	//---------------------------------------------------------------------------

	private PredicatesDomains predDomains;
	private CarContextFactory carContextFactory;
	private TrafficFactFactory factFactory;

	private List<Car> allCars, travelingCars, carsToRemove, potentialconflictingCars, travelingNorth, 
	                  travelingSouth, travelingEast, travelingWest, conflictingCars, queueE, queueW, queueN, queueS;
	private List<Long> conflictingids;
	private LinkedList<Car> availableCars;
	private List<Collision> collisions, collisionsToRemove;
	private List<Conflicting> conflicts, conflictsToRemove;
	private Map<String, GridPoint> positionsToCheck;
	private Map<Car, Direction> carsandHeadings;

	private Context<TrafficElement> context = null;
	private Grid<TrafficElement> map = null;



	private int xDim = 0;
	private int yDim = 0;
	private long lastCarEmitTickSN = 1l;
	private long lastCarEmitTickNS=1l;
	private long lastCarEmitTickEW = 1l;
	private long lastCarEmitTickWE = 1l;
	private int lowerLane = 0;
	private int upperLane = 0;
	private int leftLane = 0;
	private int rightLane = 0;
	private int freqNS=1;
	private int freqEW=1;
	private int freqSN=1;
	private int freqWE=1;

	/**
	 * Traffic view dimensions
	 */
	private int startRow, stopRow, startCol, stopCol;
	
	private Map<Mykey<CarAction, CarAction, CarAction> ,Double> payoff1;
	private Map<Mykey<CarAction, CarAction, CarAction> ,Double> payoff3;
	private Map<Pair<CarAction, CarAction> ,Double> payoff2;
	
	QLearning qlearningE;
	QLearning qlearningW;
	QLearning qlearningN;
	QLearning qlearningS;
	
	File fileelG = new File ("carontheEleftG.txt");
	File fileelS = new File ("carontheEleftS.txt");
	File fileerG = new File ("carontherErightG.txt");
	File fileerS = new File ("carontherErightS.txt");
	File fileebG = new File ("carontheEbothG.txt");
	File fileebS = new File ("carontheEbothS.txt");
	
	File filewlG = new File ("carontheWleftG.txt");
	File filewlS = new File ("carontheWleftS.txt");
	File filewrG = new File ("carontheWrightG.txt");
	File filewrS = new File ("carontheWrightS.txt");
	File filewbG = new File ("carontheWbothG.txt");
	File filewbS = new File ("carontheWbothS.txt");

	File filenlG = new File ("carontheNleftG.txt");
	File filenlS = new File ("carontheNleftS.txt");
	File filenrG = new File ("carontheNrightG.txt");
	File filenrS = new File ("carontheNrightS.txt");
	File filenbG = new File ("carontheNbothG.txt");
	File filenbS = new File ("carontheNbothS.txt");
	
	File fileslG = new File ("carontheSleftG.txt");
	File fileslS = new File ("carontheSleftS.txt");
	File filesrG = new File ("carontheSrightG.txt");
	File filesrS = new File ("carontheSrightS.txt");
	File filesbG = new File ("carontheSbothG.txt");
	File filesbS = new File ("carontheSbothS.txt");
	
	private PrintWriter outputebG;
	private PrintWriter outputebS;
	private PrintWriter outputerG;
	private PrintWriter outputerS;
	private PrintWriter outputelG;
	private PrintWriter outputelS;
	
	private PrintWriter outputwbG;
	private PrintWriter outputwbS;
	private PrintWriter outputwrG;
	private PrintWriter outputwrS;
	private PrintWriter outputwlG;
	private PrintWriter outputwlS;
	
	private PrintWriter outputsbG;
	private PrintWriter outputsbS;
	private PrintWriter outputsrG;
	private PrintWriter outputsrS;
	private PrintWriter outputslG;
	private PrintWriter outputslS;
	
	private PrintWriter outputnbG;
	private PrintWriter outputnbS;
	private PrintWriter outputnrG;
	private PrintWriter outputnrS;
	private PrintWriter outputnlG;
	private PrintWriter outputnlS;
	//---------------------------------------------------------------------------
	// Constructors 
	//---------------------------------------------------------------------------

	/**
	 * Constructor
	 *  
	 * @param context
	 * @param grid
	 * @param normLayer
	 */
	public CarMap(Context<TrafficElement> context, Grid<TrafficElement> map,
			PredicatesDomains predDomains, CarContextFactory carContextFactory,
			TrafficFactFactory factFactory) {

		super(map.getDimensions().getHeight(), map.getDimensions().getWidth());

		this.predDomains = predDomains;
		this.carContextFactory = carContextFactory;
		this.factFactory = factFactory;

		this.xDim = map.getDimensions().getWidth();
		this.yDim = map.getDimensions().getHeight();
		this.context = context;
		this.map = map;

		this.availableCars = new LinkedList<Car>();
		this.travelingCars = new ArrayList<Car>();
		this.travelingNorth = new ArrayList<Car>();
		this.travelingSouth = new ArrayList<Car>();
		this.travelingEast = new ArrayList<Car>();
		this.travelingWest = new ArrayList<Car>();
		this.conflictingCars = new ArrayList<Car>();
		this.queueE = new ArrayList<Car>();
		this.queueW = new ArrayList<Car>();
		this.queueS = new ArrayList<Car>();
		this.queueN = new ArrayList<Car>();
		this.allCars = new ArrayList<Car>();
		this.conflictingids= new ArrayList<Long>();
		this.carsToRemove = new ArrayList<Car>();
		this.potentialconflictingCars= new ArrayList<Car>();
		this.collisions = new ArrayList<Collision>();
		this.collisionsToRemove = new ArrayList<Collision>();
		this.conflicts = new ArrayList<Conflicting>();
		this.conflictsToRemove = new ArrayList<Conflicting>();
		this.positionsToCheck = new HashMap<String,GridPoint>();
		this.carsandHeadings= new  HashMap<Car, Direction>();

		this.leftLane =  (int)(Math.floor(0.5*xDim))-1;
		this.rightLane = leftLane+1;
		this.lowerLane = (int)(Math.floor(0.5*yDim))-1;
		this.upperLane = lowerLane+1;

		this.startRow = 0;
		this.stopRow = yDim-1;
		this.startCol = 0;
		this.stopCol = xDim-1;
		
		carContextFactory.set(xDim, yDim);

		this.generateCars();
		
    this.payoff1= new HashMap<Mykey<CarAction,CarAction, CarAction>, Double>();
    this.payoff3= new HashMap<Mykey<CarAction,CarAction, CarAction>, Double>();
    this.payoff2= new HashMap<Pair<CarAction,CarAction>, Double>();
    
		this.payoff();
		
		qlearningE= new QLearning();
		qlearningW= new QLearning();
		qlearningN= new QLearning();
		qlearningS= new QLearning();
		
	  this.openoutputfiles();
	  
	}



	//----------------------------------------------------------
	// Methods 
	//----------------------------------------------------------

	/**
	 * Generates the collection of cars to use
	 */
	public void generateCars() {
		Car car;

		for(short i=1; i<100; i++)
		{
			car = new Car(i, true, predDomains, carContextFactory, factFactory);
			allCars.add(car);
			availableCars.add(car);
			
		}
	}

	//------------------------------------------------------------------
	// 
	//------------------------------------------------------------------

	/**
	 * Makes all the works that must be done by the map in a step
	 * 
	 * @return
	 */
	public void step()
	{
		if (!conflictingCars.isEmpty()){
			for (Car car:conflictingCars){
				conflictingids.add(car.getId());
			}
		}
		
		for(Car car : this.travelingCars)
		{	
			if (!conflictingids.isEmpty()){
				car.perceiveAndReasonmod(conflictingids);
			}
			else if (conflictingids.isEmpty()){
				car.perceiveAndReasonmod(null);
			}
		}
		
		if (!conflictingCars.isEmpty()){
			this.getreward();
		}
				
    this.adjustCarAction();		
		this.moveCars();
		this.emitCars();
		this.codify();
		
		queueW.clear();
		queueE.clear();
		queueS.clear();
		queueN.clear();
		conflictingCars.clear();
		conflictingids.clear();
		this.cleaning();
		
		System.out.println("actualmoving");
		System.out.println(this.toString());
	}
	
	private void cleaning() {
		
		if (!this.travelingEast.isEmpty()){
			List<Car> travelingEast1= new ArrayList<Car>();
			for (Car car: this.travelingEast){
				if (car.getX()>8){
					travelingEast1.add(car);
				}
			}
			if(!travelingEast1.isEmpty()){
				this.travelingEast.removeAll(travelingEast1);
			}
		}
		
		if (!this.travelingWest.isEmpty()){
			List<Car> travelingWest1= new ArrayList<Car>();
			for (Car car: this.travelingWest){
				if (car.getX()<11){
					travelingWest1.add(car);
				}
			}
			if(!travelingWest1.isEmpty()){
				this.travelingWest.removeAll(travelingWest1);
			}
		}
		
		if (!this.travelingNorth.isEmpty()){
			List<Car> travelingNorth1= new ArrayList<Car>();
			for (Car car: this.travelingNorth){
				if (car.getY()>8){
					travelingNorth1.add(car);
				}
			}
			if(!travelingNorth1.isEmpty()){
				this.travelingNorth.removeAll(travelingNorth1);
			}
		}
		
		if (!this.travelingSouth.isEmpty()){
			List<Car> travelingSouth1= new ArrayList<Car>();
			for (Car car: this.travelingSouth){
				if (car.getY()<11){
					travelingSouth1.add(car);
				}
			}
			if(!travelingSouth1.isEmpty()){
				this.travelingSouth.removeAll(travelingSouth1);
			}
		}
		
	}

	private void getreward() {

		CarAction actionE=null;
		CarAction actionW=null;
		CarAction actionS=null;
		CarAction actionN=null;
		double rewardN=0.0d;
		double rewardS=0.0d;
		double rewardW=0.0d;
		double rewardE=0.0d;
		
		boolean travE=false;
		boolean travW=false;
		boolean travS=false;
		boolean travN=false;
		
		System.out.println("conflictingids");
		for (Car car: this.conflictingCars){
			System.out.println(car.getId());
		}
		
		for(Car car: this.conflictingCars){
			
			if (travelingEast.contains(car)){
				travE=true;
				actionE=car.getNextAction();
				System.out.println("travelingEast");
			}
			else if (travelingWest.contains(car)){
				travW=true;
				actionW=car.getNextAction();
				System.out.println("travelingWest");
		  }
			else if (travelingSouth.contains(car)){
				travS=true;
				actionS=car.getNextAction();
				System.out.println("travelingSouth");
		  }
			else if (travelingNorth.contains(car)){
			  travN=true;
			  actionN=car.getNextAction();
			  System.out.println("travelingNorth");
		  }
		}
		//
		if (travE & travN & travS){
			rewardE= payoff1.get(new Mykey<CarAction, CarAction, CarAction>(actionE,actionN,actionS));
			for (Car car: conflictingCars){
				if (travelingEast.contains(car)){
					car.rewarding(rewardE);
					System.out.print("rewardE1: ");
					System.out.print(actionE.toString());
					System.out.print(actionN.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardE);
				}
			}
		}
		if (travE & travN & !travS){
			rewardE=payoff2.get(new Pair<CarAction, CarAction>(actionE,actionN));
			for (Car car: conflictingCars){
				if (travelingEast.contains(car)){
					car.rewarding(rewardE);
					System.out.print("rewardE2: ");
					System.out.print(actionE.toString());
					System.out.print(actionN.toString());
					System.out.println(rewardE);
				}
			}
		}
		if (travE & !travN & travS){
			rewardE=payoff2.get(new Pair<CarAction, CarAction>(actionE,actionS));
			for (Car car: conflictingCars){
				if (travelingEast.contains(car)){
					car.rewarding(rewardE);
					System.out.print("rewardE3: ");
					System.out.print(actionE.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardE);
				}
			}
		}
		
		//
		if (travW & travN & travS){
			rewardW= payoff1.get(new Mykey<CarAction, CarAction, CarAction>(actionW,actionN,actionS));
			for (Car car: conflictingCars){
				if (travelingWest.contains(car)){
					car.rewarding(rewardW);
					System.out.print("rewardW1: ");
					System.out.print(actionW.toString());
					System.out.print(actionN.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardW);
				}
			}
		}
		if (travW & travN & !travS){
			rewardW=payoff2.get(new Pair<CarAction, CarAction>(actionW,actionN));
			for (Car car: conflictingCars){
				if (travelingWest.contains(car)){
					car.rewarding(rewardW);
					System.out.print("rewardW2: ");
					System.out.print(actionW.toString());
					System.out.print(actionN.toString());
					System.out.println(rewardW);
				}
			}
		}
		if (travW & !travN & travS){
			rewardW=payoff2.get(new Pair<CarAction, CarAction>(actionW,actionS));
			for (Car car: conflictingCars){
				if (travelingWest.contains(car)){
					car.rewarding(rewardW);
					System.out.print("rewardW3: ");
					System.out.print(actionW.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardW);
				}
			}
		}
		//
		
		if (travN & travE & travW){
			rewardN= payoff3.get(new Mykey<CarAction, CarAction, CarAction>(actionN,actionE,actionW));
			for (Car car: conflictingCars){
				if (travelingNorth.contains(car)){
					car.rewarding(rewardN);
					System.out.print("rewardN1: ");
					System.out.print(actionN.toString());
					System.out.print(actionE.toString());
					System.out.print(actionW.toString());
					System.out.println(rewardN);
				}
			}
		}
		if (travN & travE & !travW){
			rewardN=payoff2.get(new Pair<CarAction, CarAction>(actionE, actionN));
			for (Car car: conflictingCars){
				if (travelingNorth.contains(car)){
					car.rewarding(rewardN);
					System.out.print("rewardN2: ");
					System.out.print(actionE.toString());
					System.out.print(actionN.toString());
					System.out.println(rewardN);
				}
			}
		}
		if (travN & !travE & travW){
			rewardN=payoff2.get(new Pair<CarAction, CarAction>(actionW, actionN));
			for (Car car: conflictingCars){
				if (travelingNorth.contains(car)){
					car.rewarding(rewardN);
					System.out.print("rewardN3: ");
					System.out.print(actionW.toString());
					System.out.print(actionN.toString());
					System.out.println(rewardN);
				}
			}
		}
		//
		if (travS & travE & travW){
			rewardS= payoff3.get(new Mykey<CarAction, CarAction, CarAction>(actionS,actionE,actionW));
			for (Car car: conflictingCars){
				if (travelingSouth.contains(car)){
					car.rewarding(rewardS);
					System.out.print("rewardS1: ");
					System.out.print(actionS.toString());
					System.out.print(actionE.toString());
					System.out.print(actionW.toString());
					System.out.println(rewardS);
				}
			}
		}
		if (travS & travE & !travW){
			rewardS=payoff2.get(new Pair<CarAction, CarAction>(actionE, actionS));
			for (Car car: conflictingCars){
				if (travelingSouth.contains(car)){
					car.rewarding(rewardS);
					System.out.print("rewardS2: ");
					System.out.print(actionE.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardS);
				}
			}
		}
		if (travS & !travE & travW){
			rewardS=payoff2.get(new Pair<CarAction, CarAction>(actionW, actionS));
			for (Car car: conflictingCars){
				if (travelingSouth.contains(car)){
					car.rewarding(rewardS);
					System.out.print("rewardS3: ");
					System.out.print(actionW.toString());
					System.out.print(actionS.toString());
					System.out.println(rewardS);
				}
			}
		}
		//
	}

	/**
	 *this has been added by mehdi and this to stop the cars in the feeder lines to move if the first line in the queue is going to stop
	 * 
	 * @return
	 */
	private void adjustCarAction() {
    
		boolean stopNorth=false;
    boolean stopSouth=false;
    boolean stopEast=false;
    boolean stopWest=false;
    
		for (Car car: this.travelingCars){
			//check the position 4 action
			if ((car.getX()==rightLane) & (car.getY()==lowerLane-1)){
				CarAction action4=car.getNextAction();
				if (action4==CarAction.Stop){
					stopNorth=true;
					}
				}
			//check the position 3 action
			if ((car.getX()==leftLane-1) & (car.getY()==lowerLane)){
				CarAction action3=car.getNextAction();
				if (action3==CarAction.Stop){
					stopEast=true;
					}
				}
			//check the position 2 action
			if ((car.getX()==rightLane+1) & (car.getY()==upperLane)){
				CarAction action2=car.getNextAction();
				if (action2==CarAction.Stop){
					stopWest=true;
					}
				}
			//check the position 1 action
			if ((car.getX()==leftLane) & (car.getY()==upperLane+1)){
				CarAction action1=car.getNextAction();
				if (action1==CarAction.Stop){
					stopSouth=true;
					}
				}
			}
		
		if (stopNorth){
			this.getcarinqueueN(rightLane,lowerLane-1);
			if(queueN != null){
				for (Car car4:queueN){
					car4.modifyAction();
				}
			}
		}
		
		if (stopEast){
			this.getcarinqueueE(leftLane-1,lowerLane);
			if(queueE != null){
				for (Car car3: queueE){
					car3.modifyAction();
				}
			}
		}
		
		if (stopWest){
			this.getcarinqueueW(rightLane+1,upperLane);
			if(queueW != null){
				for (Car car2: queueW){
					car2.modifyAction();
				}
			}
		}
		
		if (stopSouth){
			this.getcarinqueueS(leftLane,upperLane+1);
			if(queueS != null){
				for (Car car1: queueS){
					car1.modifyAction();
				}
		  }
	   }
	}
	
	
	private void getcarinqueueN(int x, int y) {
		int y1=y;
		while (true){
			y1--;
			TrafficElement elem1 = map.getObjectAt(x,y1);
		  
			if (elem1 instanceof Car){
					addCarN((Car) elem1);
			}
			else if (elem1==null || y1==0){
				break; 
			}
		}
	}
	
	private void addCarN(Car elem1) {
		queueN.add(elem1);
		
	}

	private void getcarinqueueE(int x, int y) {
		int x1=x;
		while (true){
			x1--;
			TrafficElement elem2 = map.getObjectAt(x1, y);

			if (elem2 instanceof Car){
				addCarE((Car) elem2);
			}

			else if ((elem2==null) || x1==0){
				break; 
			}
		}
	}
	
	private void addCarE(Car elem2) {
		queueE.add(elem2);
		
	}

	private void getcarinqueueW(int x, int y) {
		int x1=x;
		while (true){
			x1++;
			TrafficElement elem3 = map.getObjectAt(x1,y);

			if (elem3 instanceof Car){
				addCarW((Car) elem3);
			}

			else if ((elem3==null) || x1==xDim-1){
				break; 
			}
		}
	}
	
	private void addCarW(Car elem3) {
		queueW.add(elem3);
		
	}

	private void getcarinqueueS(int x, int y) {
		int y1=y;
		while (true){
			y1++;
			
			TrafficElement elem4= map.getObjectAt(x,y1);
			
			if (elem4 instanceof Car){
				addCarS((Car) elem4);
				}
			
			else if ((elem4==null) || y1==yDim-1){
				break; 
			}
		}
	}

	private void addCarS(Car elem4) {
		
		queueS.add(elem4);
	}

	/**
	 * Makes all the works that must be done by the map in a predicted step, this has been added by mehdi
	 * 
	 * @return
	 */
	public void predictedstep()
	{

		this.moveCarsForward();
		this.codify();
		System.out.println("movingforward");
		System.out.println(this.toString());


	}

	/**
	 * move cars back 
	 * 
	 * @return
	 */
	public void backstep()
	{
		this.removeCollisions();
		this.removeConflicts();
		this.moveCarsBackward();
		this.codify();
		System.out.println("movingbackward");
		System.out.println(this.toString());

	}


	/**
	 * 
	 */
	private void codify()
	{
		this.clear();
		String codState;

		// Clear previous information
		this.clear();

		for(int row=startRow; row<=stopRow; row++) {
			for(int col=startCol; col<=stopCol; col++) {

				TrafficElement elem = this.getElement(row, col);

				// Create binary description and add it to the position
				codState = TrafficStateCodifier.codify(elem);  				
				this.set(row, col, codState);
			}
		}
	}


	/**
	 * Executes step method for each car and adds them to their new position
	 */
	private void moveCars()
	{
		this.carsToRemove.clear();
		this.positionsToCheck.clear();

		// Cars compute their new position
		for(Car car : travelingCars)
			car.move();

		// Move cars in the map
		for(Car car : travelingCars)
		{
			int x = car.getX();
			int y = car.getY();

			// Move car
			if(!this.isPositionOutOfBounds(x, y))
			{
				map.moveTo(car, x, y);
				this.addPositionToCheck(car.getPosition().getGridPoint());
			}
			// Car moved out of the map. Add for removal
			else {
				this.addCarToRemove(car);
			}
		}

		// Remove cars out of bounds
		for(Car car : this.carsToRemove)	{
			remove(car);
		}

		// Manage collisions
//		for(String posId : this.positionsToCheck.keySet())
//		{
//			GridPoint pos = this.positionsToCheck.get(posId);
//			int x = pos.getX();
//			int y = pos.getY();
//
//			if(this.getNumElements(x, y) > 1)
//			{
//				Collision col = new Collision(x, y, map);
//				Iterable<TrafficElement> elements = map.getObjectsAt(x,y);
//				remove(elements);
//
//				context.add(col);
//				map.moveTo(col, x, y);
//
//				this.collisions.add(col);
//			}
//		}
	}


	/**
	 * move cars forward for prediction
	 */	
	private void moveCarsForward() {
		
		Random rand = TrafficSimulator.getRandom();
		boolean firstzone=false, secondzone=false, thirdzone=false, forthzone=false;
		boolean firstzoneacci=false, secondzoneacci=false, thirdzoneacci=false, forthzoneacci=false;
		
		this.positionsToCheck.clear();

		// Cars compute their new position
		for(Car car : travelingCars)
			car.movef();

		// Move cars in the map
		for(Car car : travelingCars)
		{
			int x = car.getX();
			int y = car.getY();

			// Move car
			if(!this.isPositionOutOfBounds(x, y))
			{
				map.moveTo(car, x, y);
				this.addPositionToCheck(car.getPosition().getGridPoint());
				
				if (x==leftLane & y==upperLane & travelingSouth.contains(car)){
					firstzone=true;
					//conflictingCars.add(car);
				}
				else if (x==rightLane & y==upperLane & travelingWest.contains(car)){
					secondzone=true;
					//conflictingCars.add(car);
				}
				else if (x==leftLane & y==lowerLane & travelingEast.contains(car)){
					thirdzone=true;
					//conflictingCars.add(car);
				}
				else if (x==rightLane & y==lowerLane & travelingNorth.contains(car)){
					forthzone=true;
					//conflictingCars.add(car);
				}
			}

		}


		// Manage collisions
		for(String posId : this.positionsToCheck.keySet())
		{
			GridPoint pos = this.positionsToCheck.get(posId);
			int x = pos.getX();
			int y = pos.getY();

			if(this.getNumElements(x, y) > 1)
			{
				Collision col = new Collision(x, y, map);

				context.add(col);
				map.moveTo(col, x, y);
				this.collisions.add(col);
				
				if (x==leftLane & y==upperLane){
					firstzoneacci=true;
				}
				else if (x==rightLane & y==upperLane){
					secondzoneacci=true;
				}
				else if (x==leftLane & y==lowerLane ){
					thirdzoneacci=true;
				}
				else if (x==rightLane & y==lowerLane){
					forthzoneacci=true;
				}
				
			}
		}

//		 Manage conflict
		
		if (firstzone & secondzone & thirdzone & forthzone & !firstzoneacci & !secondzoneacci & !thirdzoneacci & !forthzoneacci){
				Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
				Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);
				
				context.add(conf1);
			  map.moveTo(conf1, leftLane, upperLane);
			  this.conflicts.add(conf1);
			  conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));
			  
				context.add(conf4);
				map.moveTo(conf4, rightLane, lowerLane);
				this.conflicts.add(conf4);
				conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));
				
				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);
				
				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));

				context.add(conf3);
				map.moveTo(conf3, leftLane, lowerLane);
				this.conflicts.add(conf3);
				conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));
				
		}
			
		
		else if (firstzone & secondzone & thirdzone & (!forthzone || forthzoneacci) & !firstzoneacci & !secondzoneacci & !thirdzoneacci){
			
				Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
				context.add(conf1);
			  map.moveTo(conf1, leftLane, upperLane);
			  this.conflicts.add(conf1);
			  conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));

			  
				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);

				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));

				context.add(conf3);
				map.moveTo(conf3, leftLane, lowerLane);
				this.conflicts.add(conf3);
				conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));

		}
				
		else if (firstzone & secondzone & (!thirdzone || thirdzoneacci) & forthzone & !firstzoneacci & !secondzoneacci & !forthzoneacci){
			
				Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
				Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);
				
				context.add(conf1);
			  map.moveTo(conf1, leftLane, upperLane);
			  this.conflicts.add(conf1);
			  conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));
			  
				context.add(conf4);
				map.moveTo(conf4, rightLane, lowerLane);
				this.conflicts.add(conf4);
				conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));
			  
			
				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				
				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));

		}
		else if (firstzone & (!secondzone || secondzoneacci) & thirdzone  & forthzone & !firstzoneacci & !thirdzoneacci & !forthzoneacci){
			
			Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
			Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);
			
			context.add(conf1);
		  map.moveTo(conf1, leftLane, upperLane);
		  this.conflicts.add(conf1);
		  conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));
		  
			context.add(conf4);
			map.moveTo(conf4, rightLane, lowerLane);
			this.conflicts.add(conf4);
			conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));
		  
		
			Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);
			
			context.add(conf3);
			map.moveTo(conf3, leftLane, lowerLane);
			this.conflicts.add(conf3);
			conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));

	}
		else if (firstzone & secondzone & (!thirdzone || thirdzoneacci) & (!forthzone || forthzoneacci) & !firstzoneacci & !secondzoneacci){
			
				Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
				
				context.add(conf1);
				map.moveTo(conf1, leftLane, upperLane);
				this.conflicts.add(conf1);
				conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));

				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				
				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));
				
				
		}		
	
		else if (firstzone & (!secondzone || secondzoneacci) & thirdzone & (!forthzone || forthzoneacci) & !firstzoneacci & !thirdzoneacci){
			
				Conflicting conf1 = new Conflicting (leftLane, upperLane, map);
				
				context.add(conf1);
				map.moveTo(conf1, leftLane, upperLane);
				this.conflicts.add(conf1);
				conflictingCars.add((Car) map.getObjectAt(leftLane,upperLane));

				Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);
				
				context.add(conf3);
				map.moveTo(conf3, leftLane, lowerLane);
				this.conflicts.add(conf3);
				conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));
	
		}
	
		else if ((!firstzone || firstzoneacci) & secondzone & thirdzone & forthzone & !secondzoneacci & !thirdzoneacci & !forthzoneacci){
			
				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);
				
				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));

				context.add(conf3);
				map.moveTo(conf3, leftLane, lowerLane);
				this.conflicts.add(conf3);
				conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));
		
				Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);

				context.add(conf4);
				map.moveTo(conf4, rightLane, lowerLane);
				this.conflicts.add(conf4);
				conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));

		}
		
		else if ((!firstzone || firstzoneacci) & (!secondzone || secondzoneacci) & thirdzone & forthzone & !thirdzoneacci & !forthzoneacci){
			
				Conflicting conf3 = new Conflicting (leftLane, lowerLane, map);
				context.add(conf3);
				map.moveTo(conf3, leftLane, lowerLane);
				this.conflicts.add(conf3);
				conflictingCars.add((Car) map.getObjectAt(leftLane, lowerLane));
				

				Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);
				context.add(conf4);
				map.moveTo(conf4, rightLane, lowerLane);
				this.conflicts.add(conf4);
				conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));
		}
		
		else if ((!firstzone || firstzoneacci) & secondzone & (!thirdzone || thirdzoneacci) & forthzone & !secondzoneacci & !forthzoneacci){
			
				Conflicting conf2 = new Conflicting (rightLane, upperLane, map);
				context.add(conf2);
				map.moveTo(conf2, rightLane, upperLane);
				this.conflicts.add(conf2);
				conflictingCars.add((Car) map.getObjectAt(rightLane, upperLane));

				Conflicting conf4 = new Conflicting (rightLane, lowerLane, map);
				context.add(conf4);
				map.moveTo(conf4, rightLane, lowerLane);
				this.conflicts.add(conf4);
				conflictingCars.add((Car) map.getObjectAt(rightLane, lowerLane));
			
		}
		
	}


	/**
	 * move cars backward in their original position
	 */	
	private void moveCarsBackward() {

		// Cars compute their previous position
		for(Car car : travelingCars)
			car.moveb();

		// Move cars in the map
		for(Car car : travelingCars)
		{
			int x = car.getX();
			int y = car.getY();

			// Move car
			if(!this.isPositionOutOfBounds(x, y))
			{
				map.moveTo(car, x, y);
				//				this.addPositionToCheck(car.getPosition().getGridPoint());
			}
		}

	}



	/**
	 * Emits new cars
	 */
	public void emitCars() {
		int numAddedCars = 0;
		int numAddedCarsEW = 0;
		int numAvailableCars = availableCars.size();

		// Emit cars every N steps for south north
//		if((TrafficSimulator.getTick() == (long)(this.lastCarEmitTick +
//				(long)TrafficNormSynthesisSettings.SIM_NEW_CARS_FREQUENCY)
//				)) {
		if((TrafficSimulator.getTick() == (long)(this.lastCarEmitTickSN +
				freqSN)
				)) {			

			int carsToEmit = Math.min(
					TrafficNormSynthesisSettings.SIM_NUM_CARS_TO_EMIT, 2);

			CarPosition cp = null;

			this.lastCarEmitTickSN = TrafficSimulator.getTick();
			freqSN=this.getrandomSN();

			while(numAvailableCars > 0 && numAddedCars < carsToEmit){
				cp = getFreeRandomStartPointSN();
				numAddedCars++;

				// Cancel, since no starting points are free
				if(cp == null)
					break;  
				else {
					add(cp);
					//					System.out.println("    > Car emited");
				}
			}		
		}
	// Emit cars every N steps for north south
		if((TrafficSimulator.getTick() == (long)(this.lastCarEmitTickNS +
				freqNS)
				)) {			

			int carsToEmit = Math.min(
					TrafficNormSynthesisSettings.SIM_NUM_CARS_TO_EMIT, 2);

			CarPosition cp1 = null;

			this.lastCarEmitTickNS = TrafficSimulator.getTick();
			freqNS=this.getrandomNS();

			while(numAvailableCars > 0 && numAddedCars < carsToEmit){
				cp1 = getFreeRandomStartPointNS();
				numAddedCars++;

				// Cancel, since no starting points are free
				if(cp1 == null)
					break;  
				else {
					add(cp1);
					//					System.out.println("    > Car emited");
				}
			}		
		}
		// Emit cars every N steps for east west
		if((TrafficSimulator.getTick() == (long)(this.lastCarEmitTickEW +
				freqEW)
				)) {


			int carsToEmit = Math.min(
					TrafficNormSynthesisSettings.SIM_NUM_CARS_TO_EMIT, 2);

			CarPosition cp2 = null;

			this.lastCarEmitTickEW = TrafficSimulator.getTick();
			freqEW=this.getrandomEW();

			while(numAvailableCars > 0 && numAddedCarsEW < carsToEmit){
				cp2 = getFreeRandomStartPointEW();
				numAddedCars++;

				// Cancel, since no starting points are free
				if(cp2 == null)
					break;  
				else {
					add(cp2);
					//					System.out.println("    > Car emited");
				}
			}		
		}
		
		// Emit cars every N steps for  west east
		if((TrafficSimulator.getTick() == (long)(this.lastCarEmitTickWE +
				freqWE)
				)) {


			int carsToEmit = Math.min(
					TrafficNormSynthesisSettings.SIM_NUM_CARS_TO_EMIT, 2);

			CarPosition cp3 = null;

			this.lastCarEmitTickWE = TrafficSimulator.getTick();
			freqWE=this.getrandomWE();

			while(numAvailableCars > 0 && numAddedCarsEW < carsToEmit){
				cp3 = getFreeRandomStartPointWE();
				numAddedCars++;

				// Cancel, since no starting points are free
				if(cp3 == null)
					break;  
				else {
					add(cp3);
					//					System.out.println("    > Car emited");
				}
			}		
		}
	}

	/**
	 * Adds a list of norms to all the cars in the simulation
	 * 
	 * @param norms
	 */
	public void broadcastAddNorm(Norm norm) {
		for(Car c : allCars) {
			c.getReasoner().addNorm(norm);
		}
	}

	/**
	 * Adds a list of norms to all the cars in the simulation
	 * 
	 * @param norms
	 */
	public void broadcastRemoveNorm(Norm norm) {
		for(Car c : allCars) {
			c.getReasoner().removeNorm(norm);
		}
	}

	//----------------------------------------------------------
	// Add and remove
	//----------------------------------------------------------

	/**
	 * 
	 * @param pos
	 */
	private void add(CarPosition pos)
	{
		Car car = availableCars.pop();
		car.init(pos);
		add(car);
		carsandHeadings.put(car, pos.getDirection());

	}

	/**
	 * Adds a car to the simulation
	 * 
	 * @param car
	 */
	private void add(Car car)
	{
		travelingCars.add(car);
		context.add(car);
		map.moveTo(car, car.getX(), car.getY());
		
		if (car.getPosition().getDirection()==Direction.North & !travelingNorth.contains(car)){
			travelingNorth.add(car);
		}
		
		else if (car.getPosition().getDirection()==Direction.South & !travelingSouth.contains(car)){
			travelingSouth.add(car);
		}
		
		else if (car.getPosition().getDirection()==Direction.East & !travelingEast.contains(car)){
			travelingEast.add(car);
		}
		
		else if (car.getPosition().getDirection()==Direction.West & !travelingWest.contains(car)){
			travelingWest.add(car);
		}
		
	}

	/**
	 * 
	 * @param pos
	 */
	private void addPositionToCheck(GridPoint pos)
	{
		String s = pos.getX() + "-" + pos.getY();
		this.positionsToCheck.put(s, pos);
	}

	/**
	 * 
	 * @param elements
	 */
	private void remove(Iterable<TrafficElement> elements)
	{
		List<TrafficElement> toRemove = new ArrayList<TrafficElement>();

		for(TrafficElement element : elements)
			toRemove.add(element);

		for(TrafficElement element : toRemove)
			remove(element);			
	}


	/**
	 * 
	 * @param element
	 */
	private void remove(TrafficElement element)
	{
		context.remove(element);

		if(element instanceof Car) {
			removeCar((Car)element);
		}
	}


	/**
	 * Removes a car from the simulation and its position
	 * 
	 * @param car
	 */
	private void removeCar(Car car)
	{
		travelingCars.remove(car);
		availableCars.addLast(car);
	}

	/**
	 * Removes collided cars from the car map
	 */
	public void removeCollisions()
	{
		this.collisionsToRemove.clear();

		for(Collision col : this.collisions) {
			this.collisionsToRemove.add(col);
		}

		for(Collision col : this.collisionsToRemove) {
			this.removeCollision(col);
		}
	}

	/**
	 * 
	 * @param col
	 */
	private void removeCollision(Collision col)
	{
		context.remove(col);
		this.collisions.remove(col);
	}
	
	/**
	 * Removes conflicted cars from the car map
	 */
	public void removeConflicts()
	{
		this.conflictsToRemove.clear();

		for(Conflicting conf : this.conflicts) {
			this.conflictsToRemove.add(conf);
		}

		for(Conflicting conf : this.conflictsToRemove) {
			this.removeConflict(conf);
		}
	}

	private void removeConflict(Conflicting conf) {
		context.remove(conf);
		this.conflicts.remove(conf);
	}

	/**
	 * 
	 * @param car
	 */
	private void addCarToRemove(Car car)
	{
		this.carsToRemove.add(car);
	}

	//----------------------------------------------------------
	// Getters and setters
	//----------------------------------------------------------

	/**
	 * 
	 * @param x
	 * @param y
	 * @return
	 */
	private int getNumElements(int x, int y)
	{
		Iterable<TrafficElement> elements = map.getObjectsAt(x,y);
		int elemsCount = 0;

		Iterator<TrafficElement> iterator = elements.iterator();
		while(iterator.hasNext()) {
			iterator.next();
			elemsCount++;
		}
		return elemsCount;
	}

	/**
	 * 
	 * @param row
	 * @param col
	 * @return
	 */
	public TrafficElement getElement(int row, int col) {
		TrafficElement elemd= this.map.getObjectAt(col, yDim-1-row);
		Iterable<TrafficElement> elements=this.map.getObjectsAt(col,yDim-1-row);
		if (elements !=null){
			for (TrafficElement elem: elements){
				if (elem instanceof Collision){
					return elem;
				}
				else if (elem instanceof Conflicting){
					return elem;
				}
			}
		}
		return elemd;
	}

	/**
	 * 
	 * @param id
	 * @return
	 */
	public Car getCar(long id) {
		for(Car car : travelingCars) {
			if(car.getId() == id)
				return car;
		}
		return null;
	}

	/**
	 * Returns the number of cars currently driving into the scenario
	 * 
	 * @return
	 */
	public int getNumCars() {
		return travelingCars.size();
	}

	/**
	 * Returns the start point in the map for a given direction
	 */
	private CarPosition getStartPoint(Direction dir){
		int tx = 0,ty = 0;
		switch(dir)
		{
		case North:
			tx = leftLane;
			ty = yDim-1;
			break;
		case East:
			tx = xDim-1;
			ty = upperLane;
			break;
		case South:
			tx = rightLane;
			ty = 0;
			break;
		case West: 
			tx = 0;
			ty = lowerLane;
			break;
		}
		CarPosition cp = new CarPosition(tx,ty,dir.getOppositeDirection());
		return cp;
	}

	/**
	 * Returns a free random start point in the map
	 * 
	 * @return
	 */
	private CarPosition getFreeRandomStartPointNS()
	{
		//Direction dir = Utilities.getRandomDirection();
		Direction dir= Direction.North;
		CarPosition cp = getStartPoint(dir);
		int cnt = 0;

//		while(!isFree(cp.getGridPoint()) && cnt++ < 1) {
//			cp = getStartPoint(dir = Direction.South);
//		}
		if(!isFree(cp.getGridPoint()))
			cp = null;

		return cp;
	}
	
	private CarPosition getFreeRandomStartPointSN()
	{
		//Direction dir = Utilities.getRandomDirection();
		Direction dir= Direction.South;
		CarPosition cp = getStartPoint(dir);
//		int cnt = 0;

//		while(!isFree(cp.getGridPoint()) && cnt++ < 1) {
//			cp = getStartPoint(dir = Direction.South);
//		}
		if(!isFree(cp.getGridPoint()))
			cp = null;

		return cp;
	}

	private CarPosition getFreeRandomStartPointEW()
	{
		//Direction dir = Utilities.getRandomDirection();
		Direction dir= Direction.East;
		CarPosition cp = getStartPoint(dir);
//		int cnt = 0;

//		while(!isFree(cp.getGridPoint()) && cnt++ < 1) {
//			cp = getStartPoint(dir = Direction.West);
//		}
		if(!isFree(cp.getGridPoint()))
			cp = null;

		return cp;
	}
	
	private CarPosition getFreeRandomStartPointWE()
	{
		//Direction dir = Utilities.getRandomDirection();
		Direction dir= Direction.West;
		CarPosition cp = getStartPoint(dir);
//		int cnt = 0;

//		while(!isFree(cp.getGridPoint()) && cnt++ < 1) {
//			cp = getStartPoint(dir = Direction.West);
//		}
		if(!isFree(cp.getGridPoint()))
			cp = null;

		return cp;
	}

	/**
	 * Returns true if the car is out of the map
	 * 
	 * @param p
	 * @return
	 */
	public boolean isPositionOutOfBounds(int x, int y)
	{
		if(x<0 || y < 0 || x >= xDim || y >= yDim)
			return true;
		return false;
	}

	/**
	 * Returns true if a grid point of the map is free (with no car) 
	 * 
	 * @param p
	 * @return
	 */
	private boolean isFree(GridPoint p)
	{
		int count = 0;
		Iterable<TrafficElement> elements = map.getObjectsAt(p.getX(),p.getY());
		for(TrafficElement elem : elements) {
			count++;
		}
		return count==0;
	}

	/**
	 * Returns the x dimension of the map
	 * 
	 * @return  the xDim
	 */
	public int getXDim() 
	{
		return xDim;
	}

	/**
	 * Returns the y dimension of the map
	 * 
	 * @return  the yDim
	 */
	public int getYDim() 
	{
		return yDim;
	}

	/**
	 * Returns the lower lane of the map
	 * 
	 * @return  the lowerLane
	 */
	public int getLowerLane() 
	{
		return lowerLane;
	}

	/**
	 * Returns the upper lane of the map
	 * 
	 * @return  the upperLane
	 */
	public int getUpperLane() 
	{
		return upperLane;
	}

	/**
	 * Returns the left lane of the map
	 * 
	 * @return  the leftLane
	 */
	public int getLeftLane() 
	{
		return leftLane;
	}

	/**
	 * Returns the right lane of the map
	 * 
	 * @return  the rightLane
	 */
	public int getRightLane()
	{
		return rightLane;
	}
	
	public void payoff(){
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Go, CarAction.Go), -5.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Stop, CarAction.Go), -2.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Go, CarAction.Stop), -2.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Stop, CarAction.Stop), 0.5);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Go, CarAction.Go), 4.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Stop, CarAction.Go), 1.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Go, CarAction.Stop), 1.0);
		payoff1.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Stop, CarAction.Stop), 0.0);
		
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Go, CarAction.Go), -5.0);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Stop, CarAction.Go), -2.0);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Go, CarAction.Stop), -2.0);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Go,CarAction.Stop, CarAction.Stop), 4.0);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Go, CarAction.Go), 0.5);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Stop, CarAction.Go), 0.5);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Go, CarAction.Stop), 0.5);
		payoff3.put(new Mykey<CarAction, CarAction, CarAction> (CarAction.Stop,CarAction.Stop, CarAction.Stop), 0.0);

		payoff2.put(new Pair<CarAction, CarAction>(CarAction.Go,CarAction.Go), -5.0);
		payoff2.put(new Pair<CarAction, CarAction>(CarAction.Go,CarAction.Stop), 0.5);
		payoff2.put(new Pair<CarAction, CarAction>(CarAction.Stop,CarAction.Go), 4.0);
		payoff2.put(new Pair<CarAction, CarAction>(CarAction.Stop,CarAction.Stop), 0.0);
		
	}
	
	public double getQValue(Car car, SetOfPredicatesWithTerms precondition,
			CarAction action){
		if (travelingEast.contains(car)){
			if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//				outputebG.print(precondition);
//				outputebG.print(" Qvalue:");
				outputebG.print(TrafficSimulator.tick);
				outputebG.print(" ");
				outputebG.println(qlearningE.getQvalue(precondition, action));
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputebS = new PrintWriter(fileebS);
//				outputebS.print(precondition);
				outputebS.print(TrafficSimulator.tick);
				outputebS.print(" ");
				outputebS.println(qlearningE.getQvalue(precondition, action));
//				outputebS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Go){
//			try{
//				PrintWriter outputelG = new PrintWriter(fileelG);
//				outputelG.print(precondition);
				outputelG.print(TrafficSimulator.tick);
				outputelG.print(" ");
				outputelG.println(qlearningE.getQvalue(precondition, action));
//				outputelG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputelS = new PrintWriter(fileelS);
//				outputelS.print(precondition);
				outputelS.print(TrafficSimulator.tick);
				outputelS.print(" ");
				outputelS.println(qlearningE.getQvalue(precondition, action));
//				outputelS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputerG = new PrintWriter(fileerG);
//				outputerG.print(precondition);
				outputerG.print(TrafficSimulator.tick);
				outputerG.print(" ");
				outputerG.println(qlearningE.getQvalue(precondition, action));
//				outputerG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputerS = new PrintWriter(fileerS);
//				outputerS.print(precondition);
				outputerS.print(TrafficSimulator.tick);
				outputerS.print(" ");
				outputerS.println(qlearningE.getQvalue(precondition, action));
//				outputerS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			return qlearningE.getQvalue(precondition, action);
		}
		
		
		else if (travelingWest.contains(car)){
			if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputwbG = new PrintWriter(filewbG);
//				outputwbG.print(precondition);
				outputwbG.print(TrafficSimulator.tick);
				outputwbG.print(" ");
				outputwbG.println(qlearningW.getQvalue(precondition, action));
//				outputwbG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputwbS = new PrintWriter(filewbS);
//				outputwbS.print(precondition);
				outputwbS.print(TrafficSimulator.tick);
				outputwbS.print(" ");
				outputwbS.println(qlearningW.getQvalue(precondition, action));
//				outputwbS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Go){
//			try{
//				PrintWriter outputwlG = new PrintWriter(filewlG);
//				outputwlG.print(precondition);
				outputwlG.print(TrafficSimulator.tick);
				outputwlG.print(" ");
				outputwlG.println(qlearningW.getQvalue(precondition, action));
//				outputwlG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputwlS = new PrintWriter(filewlS);
//				outputwlS.print(precondition);
				outputwlS.print(TrafficSimulator.tick);
				outputwlS.print(" ");
				outputwlS.println(qlearningW.getQvalue(precondition, action));
//				outputwlS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputwrG = new PrintWriter(filewrG);
//				outputwrG.print(precondition);
				outputwrG.print(TrafficSimulator.tick);
				outputwrG.print(" ");
				outputwrG.println(qlearningW.getQvalue(precondition, action));
//				outputwrG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputwrS = new PrintWriter(filewrS);
//				outputwrS.print(precondition);
				outputwrS.print(TrafficSimulator.tick);
				outputwrS.print(" ");
				outputwrS.println(qlearningW.getQvalue(precondition, action));
//				outputwrS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			return qlearningW.getQvalue(precondition, action);
		}
		
		else if (travelingNorth.contains(car)){
			if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputnbG = new PrintWriter(filenbG);
//				outputnbG.print(precondition);
				outputnbG.print(TrafficSimulator.tick);
				outputnbG.print(" ");
				outputnbG.println(qlearningN.getQvalue(precondition, action));
//				outputnbG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputnbS = new PrintWriter(filenbS);
//				outputnbS.print(precondition);
				outputnbS.print(TrafficSimulator.tick);
				outputnbS.print(" ");
				outputnbS.println(qlearningN.getQvalue(precondition, action));
//				outputnbS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Go){
//			try{
//				PrintWriter outputnlG = new PrintWriter(filenlG);
//				outputnlG.print(precondition);
				outputnlG.print(TrafficSimulator.tick);
				outputnlG.print(" ");
				outputnlG.println(qlearningN.getQvalue(precondition, action));
//				outputnlG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputnlS = new PrintWriter(filenlS);
//				outputnlS.print(precondition);
				outputnlS.print(TrafficSimulator.tick);
				outputnlS.print(" ");
				outputnlS.println(qlearningN.getQvalue(precondition, action));
//				outputnlS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputnrG = new PrintWriter(filenrG);
//				outputnrG.print(precondition);
				outputnrG.print(TrafficSimulator.tick);
				outputnrG.print(" ");
				outputnrG.println(qlearningN.getQvalue(precondition, action));
//				outputnrG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputnrS = new PrintWriter(filenrS);
//				outputnrS.print(precondition);
				outputnrS.print(TrafficSimulator.tick);
				outputnrS.print(" ");
				outputnrS.println(qlearningN.getQvalue(precondition, action));
//				outputnrS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}	
			return qlearningN.getQvalue(precondition, action);
		}
		
		else {
			if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//				PrintWriter outputsbG = new PrintWriter(filesbG);
//						outputsbG.print(precondition);
				outputsbG.print(TrafficSimulator.tick);
				outputsbG.print(" ");
				outputsbG.println(qlearningS.getQvalue(precondition, action));
//				outputsbG.close();
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputsbS = new PrintWriter(filesbS);
//				outputsbS.print(precondition);
				outputsbS.print(TrafficSimulator.tick);
				outputsbS.print(" ");
				outputsbS.println(qlearningS.getQvalue(precondition, action));
//				outputsbS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Go){
//			try{
//				PrintWriter outputslG = new PrintWriter(fileslG);
//				outputslG.print(precondition);
				outputslG.print(TrafficSimulator.tick);
				outputslG.print(" ");
				outputslG.println(qlearningS.getQvalue(precondition, action));
//				outputslG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[>]") && precondition.getTerms("r").toString().equals("[-]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputslS = new PrintWriter(fileslS);
//				outputslS.print(precondition);
				outputslS.print(TrafficSimulator.tick);
				outputslS.print(" ");
				outputslS.println(qlearningS.getQvalue(precondition, action));
//				outputslS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Go){
//			try{
//				PrintWriter outputsrG = new PrintWriter(filesrG);
//				outputsrG.print(precondition);
				outputsrG.print(TrafficSimulator.tick);
				outputsrG.print(" ");
				outputsrG.println(qlearningS.getQvalue(precondition, action));
//				outputsrG.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}
			
			else if (precondition.getTerms("l").toString().equals("[-]") && precondition.getTerms("r").toString().equals("[<]") && action==CarAction.Stop){
//			try{
//				PrintWriter outputsrS = new PrintWriter(filesrS);
//				outputsrS.print(precondition);
				outputsrS.print(TrafficSimulator.tick);
				outputsrS.print(" ");
				outputsrS.println(qlearningS.getQvalue(precondition, action));
//				outputsrS.close();
//				} catch (IOException ex){
//					System.out.println("error");
//			}
			}	
			return qlearningS.getQvalue(precondition, action);
		}
	}
	
	public void updating(Car car, Pair<SetOfPredicatesWithTerms, CarAction> pair, double reward){
		if (travelingEast.contains(car)){
			qlearningE.update(pair, reward);
		}
		else if (travelingWest.contains(car)){
			qlearningW.update(pair, reward);
		}
		else if (travelingNorth.contains(car)){
			qlearningN.update(pair, reward);
		}
		else if (travelingSouth.contains(car)) {
			qlearningS.update(pair, reward);
		}
	}
	public int getrandomNS(){
		Random rand1 = TrafficSimulator.getRandom();
		return rand1.nextInt(4)+1; 
	}
	
	public int getrandomSN(){
		Random rand2 = TrafficSimulator.getRandom();
		return rand2.nextInt(4)+1; 
	}
	
	public int getrandomEW(){
		Random rand3 = TrafficSimulator.getRandom();
		return rand3.nextInt(5)+1; 
	}
	
	public int getrandomWE(){
		Random rand4 = TrafficSimulator.getRandom();
		return rand4.nextInt(5)+1; 
	}
	
	private void openoutputfiles(){
		// TODO Auto-generated method stub
		try {
			outputebG = new PrintWriter(fileebG);
			outputebS = new PrintWriter(fileebS);
			outputerG = new PrintWriter(fileerG);
			outputerS = new PrintWriter(fileerS);
			outputelG = new PrintWriter(fileelG);
			outputelS = new PrintWriter(fileelS);
			
			outputwbG = new PrintWriter(filewbG);
			outputwbS = new PrintWriter(filewbS);
			outputwrG = new PrintWriter(filewrG);
			outputwrS = new PrintWriter(filewrS);
			outputwlG = new PrintWriter(filewlG);
			outputwlS = new PrintWriter(filewlS);
			
			outputsbG = new PrintWriter(filesbG);
			outputsbS = new PrintWriter(filesbS);
			outputsrG = new PrintWriter(filesrG);
			outputsrS = new PrintWriter(filesrS);
			outputslG = new PrintWriter(fileslG);
			outputslS = new PrintWriter(fileslS);
			
			outputnbG = new PrintWriter(filenbG);
			outputnbS = new PrintWriter(filenbS);
			outputnrG = new PrintWriter(filenrG);
			outputnrS = new PrintWriter(filenrS);
			outputnlG = new PrintWriter(filenlG);
			outputnlS = new PrintWriter(filenlS);
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
}
