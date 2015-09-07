package es.csic.iiia.normlab.traffic.config;

import es.csic.iiia.nsm.config.Goal;

/**
 * 
 * @author "Mehdi Mashayekhi (mmashay2@ncsu.edu)"
 *
 */

public class Gconfc extends Goal {


	@Override
	public String getName() {
	  return "Gconfc";
	}

	@Override
	public String getDescription() {
		return "Avoid conflicts between cars";
	}

}
