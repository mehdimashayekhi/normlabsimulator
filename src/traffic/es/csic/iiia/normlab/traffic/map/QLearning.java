package es.csic.iiia.normlab.traffic.map;


import java.util.HashMap;
import java.util.Map;

import es.csic.iiia.normlab.traffic.car.CarAction;
import es.csic.iiia.normlab.traffic.car.Pair;
import es.csic.iiia.normlab.traffic.car.context.CarContext;
import es.csic.iiia.nsm.agent.language.SetOfPredicatesWithTerms;
import es.csic.iiia.nsm.norm.Norm;

/**
 * 
 * @author "Mehdi Mashayekhi"
 *
 */


public class QLearning {
	
	private Map<Pair<SetOfPredicatesWithTerms, CarAction>, Double> QValues;
	private double alpha=0.2d;
	


	public QLearning() {
		// TODO Auto-generated constructor stub
		QValues = new HashMap<Pair<SetOfPredicatesWithTerms, CarAction>, Double>();	
	}
	
	public CarAction getPolicy(Norm norm) {
		SetOfPredicatesWithTerms precondition=norm.getPrecondition();
  	CarAction action = (CarAction)norm.getAction();
		if (getQvalue(precondition, action)> getQvalue(precondition, action.getOpposite())){
			return action;
		}
		else {
			return action.getOpposite();
		}
	}
	
	public double getQvalue(SetOfPredicatesWithTerms precondition,
			CarAction action) {
		Pair<SetOfPredicatesWithTerms, CarAction> pair = new Pair<SetOfPredicatesWithTerms, CarAction>(precondition, action);
		if (!QValues.containsKey(pair)){
			return 0.0d;
		}
		else {
			return QValues.get(pair);
			}
	}
	
	//updating the q-values
	public void update(Pair<SetOfPredicatesWithTerms, CarAction> pair, double reward){
		if (QValues.containsKey(pair)){
			QValues.put(pair,(1-alpha)*QValues.get(pair)+alpha*(reward));
			System.out.print("pair1: ");
			System.out.println(pair.toString());
		}
		else {
			QValues.put(pair,alpha*(reward));
			System.out.print("pair2: ");
			System.out.println(pair.toString());
		}
		System.out.print("qvalue: ");
    System.out.println(QValues.get(pair));
	}

}
