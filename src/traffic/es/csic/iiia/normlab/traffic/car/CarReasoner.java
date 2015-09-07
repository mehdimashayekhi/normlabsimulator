package es.csic.iiia.normlab.traffic.car;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Random;

import es.csic.iiia.normlab.traffic.TrafficSimulator;
import es.csic.iiia.normlab.traffic.agent.Car;
import es.csic.iiia.normlab.traffic.car.context.CarContext;
import es.csic.iiia.normlab.traffic.factory.TrafficFactFactory;
import es.csic.iiia.normlab.traffic.normsynthesis.TrafficNormSynthesisSettings;
import es.csic.iiia.normlab.traffic.utils.Direction;
import es.csic.iiia.nsm.agent.language.PredicatesDomains;
import es.csic.iiia.nsm.agent.language.SetOfPredicatesWithTerms;
import es.csic.iiia.nsm.norm.Norm;
import es.csic.iiia.nsm.norm.NormModality;
import es.csic.iiia.nsm.norm.reasoning.NormEngine;

/**
 * Reasoner for the agent. Extends NormEngine to be able to reason 
 * about facts of the system and norms' applicability
 * 
 * @author "Mehdi Mashayekhi"
 *
 */
public class CarReasoner extends NormEngine {

	//------------------------------------------------------------
	// Attributes																															
	//------------------------------------------------------------

	private TrafficFactFactory factFactory;
	private CarReasonerState state;			// the state of the agent's reasoner
	private Norm normToComplyWith;			// the last norm the agent complied with
	private Norm normToInfringe;				// the last infringed norm
	private List<Norm> applicableNorms;	// norms that are applicable to the facts
	private boolean casualStop;				// is a casual stop?
	private double epsilon=0.0d;     //added by mehdi for epsilon greedy implementation
	private int n=0;                 // added by mehdi, number of iterations
	private Map<SetOfPredicatesWithTerms, Integer> seeingContext;
	public static long tick1 = 0;

	//------------------------------------------------------------
	// Constructors
	//------------------------------------------------------------

	/**
	 * Constructor
	 */
	public CarReasoner(PredicatesDomains predDomains,
			TrafficFactFactory factFactory) {
		
		super(predDomains);

		this.factFactory = factFactory;
		this.state = CarReasonerState.NoNormActivated;
		this.casualStop = false;
		this.seeingContext= new HashMap<SetOfPredicatesWithTerms, Integer>();
	}

	//---------------------------------------------------------------------------
	// Methods
	//---------------------------------------------------------------------------

	/**
	 * Does reasoning to activate rules in base of the facts in the knowledge base 
	 */
	public CarAction decideAction(Car car, CarContext context) {
		
		
		boolean violate, Random;
		int violateProb;
		this.state = CarReasonerState.NoNormActivated;

		Random rand = TrafficSimulator.getRandom();

		// Sometimes a car stops to buy tobacco or the newspaper
		//		int numTicksTraveling = myCar.getNumTicksTraveling();
		//		int numTicksToStop = myCar.getNumTicksToStop();
		//		int tickToStopAt = myCar.getTickToStopAt();
		//		
		//		if(numTicksTraveling >= tickToStopAt &&
		//				numTicksTraveling < (tickToStopAt+numTicksToStop))
		//		{
		//			this.casualStop = true;
		//			return CarAction.Stop;
		//		}

		this.reset();

		/* Add world facts */
		
		/* If the context is null, then the agent is
		 * about getting out of the scenario -> Perform action 'Go' */
		if(context == null) {
			return CarAction.Go;
		}

		SetOfPredicatesWithTerms predicates =  factFactory.generatePredicatesmod(context);
		this.addFacts(predicates);

		/* Collided cars remain stopped */
		if(car.isCollided()) {
			return CarAction.Stop;
		}

		/* Reason about the norms that apply to the given system facts */
		this.applicableNorms = this.reason();

		/* Remove random component of JESS */
		Collections.sort(applicableNorms);

		// Obtain next supposed action to do according to the norm specification
		// TODO: Maybe here in the future several norms are fired for a certain situation
		// and we will have to choose what norm to apply (in base of some criteria).
		for(Norm n : applicableNorms) {
			// norm violation is not allowed if it leads to collision (norm modality is prohibition)
			if (n.getModality()==NormModality.Prohibition){
				this.normToComplyWith = n;
				state = CarReasonerState.NormWillBeApplied;
				CarAction action = (CarAction)n.getAction();
				return action.getOpposite();
			}
		}
		for(Norm n : applicableNorms) {
//			// norm violation is not allowed if it leads to collision (norm modality is prohibition)
//			if (n.getModality()==NormModality.Prohibition){
//				this.normToComplyWith = n;
//				state = CarReasonerState.NormWillBeApplied;
//				CarAction action = (CarAction)n.getAction();
//				return action.getOpposite();
//			}
      
			if (!seeingContext.isEmpty() && seeingContext.containsKey(n.getPrecondition())){
				int m= seeingContext.get(n.getPrecondition());
				epsilon=Math.exp(-0.1*m);
				m++;
				seeingContext.put(n.getPrecondition(), m);
				System.out.print("repeating: ");
				System.out.print(n.getPrecondition().toString());
				System.out.print(": ");
				System.out.print(m);
			}
			else{
				seeingContext.put(n.getPrecondition(), 1);
				epsilon=Math.exp(-0.1*0);
				System.out.print("nonrepeating: ");
				System.out.println(n.getPrecondition().toString());
			}
			
			violate = false;
			Random=false;

			/* Decide if applying the norm or not */ 
			//violateProb = (int)(TrafficNormSynthesisSettings.SIM_NORM_VIOLATION_RATE * 100.0f);
			violateProb = (int)(TrafficNormSynthesisSettings.SIM_NORM_VIOLATION_RATE);
			//int num = rand.nextInt(100)+1;
			double num=rand.nextDouble();
			violate = (num <= violateProb) ? true : false;
			tick1++;
			if (violate && tick1 <= (TrafficNormSynthesisSettings.SIM_MAX_TICKS)/10){
				this.normToInfringe = n;
				state = CarReasonerState.NormWillBeViolated;
				return CarAction.Go;
			}
			
			else {
				double num2=rand.nextDouble();
				
				//violate = (num <= violateProb) ? true : false;
				Random = (num2 <= epsilon) ? true : false;
				
				/* Randomly choose if applying the norm or not. Case apply the norm */
				if(Random)	{
					System.out.println("random action");
					double num3=rand.nextDouble();
					if (num3<0.5){
						this.normToInfringe = n;
						state = CarReasonerState.NormWillBeViolated;
						return CarAction.Go;
					}
					else{
						this.normToComplyWith = n;
						state = CarReasonerState.NormWillBeApplied;
						CarAction action = (CarAction)n.getAction();
						return action.getOpposite();
					}
					}
				//act according to policy
				else {
					System.out.println("policy");
					CarAction action = getPolicy(n, car);
					if (action==CarAction.Go){
						this.normToInfringe = n;
						state = CarReasonerState.NormWillBeViolated;
						return action;
					}
					else if (action==CarAction.Stop){
						this.normToComplyWith = n;
						state = CarReasonerState.NormWillBeApplied;
						return action;
					}
					
//					this.normToComplyWith = n;
//					state = CarReasonerState.NormWillBeApplied;
//					CarAction action = (CarAction)n.getAction();
//					return action.getOpposite();
				}
			}
			
		}
		// Let the facts base empty and return the action chosen by the car
		applicableNorms.clear();

		return CarAction.Go;
	}

	private CarAction getPolicy(Norm norm, Car car) {
		SetOfPredicatesWithTerms precondition=norm.getPrecondition();
  	CarAction action = (CarAction)norm.getAction();
		if (getQvalue(precondition, action, car)> getQvalue(precondition, action.getOpposite(), car)){
			return action;
		}
		else {
			return action.getOpposite();
		}
	}

	private double getQvalue(SetOfPredicatesWithTerms precondition,
			CarAction action, Car car) {
		return TrafficSimulator.getMap().getQValue(car, precondition, action);
	}

	/**
	 * Returns true if the last applicable norm was finally applied. False else
	 * 
	 * @return
	 */
	public CarReasonerState getState() {
		return state;
	}

	/**
	 * Returns the last norm that has been applied by the car
	 * 
	 * @return
	 */
	public Norm getNormToApply() {
		return this.normToComplyWith;
	}

	/**
	 * Returns the last norm that has been violated by the car
	 * @return
	 */
	public Norm getNormToViolate() {
		return this.normToInfringe;
	}

	/**
	 * 
	 * @return
	 */
	public boolean isCasualStop() {
		return this.casualStop;
	}

	/**
	 * Returns the norms that apply to the facts that
	 * have been added to the norm engine
	 * 
	 * @return the list of applicable norms
	 */
	public List<Norm> getApplicableNorms() {
		return this.applicableNorms;
	}
	
	//updating the q-values
	public void update(Car car, CarContext context, CarAction action, double reward){
		SetOfPredicatesWithTerms predicates =  factFactory.generatePredicatesmod(context);
		Pair<SetOfPredicatesWithTerms, CarAction> pair = new Pair<SetOfPredicatesWithTerms, CarAction>(predicates, action);
		TrafficSimulator.getMap().updating(car, pair, reward);
	}

}
