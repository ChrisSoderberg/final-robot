package rp.robotics.localisation;

import rp.robotics.mapping.Heading;

/**
 * Example structure for an action model that should move the probabilities 1
 * cell in the requested direction. In the case where the move would take the
 * robot into an obstacle or off the map, this model assumes the robot stayed in
 * one place. This is the same as the model presented in Robot Programming
 * lecture on action models.
 * 
 * Note that this class doesn't actually do this, instead it shows you a
 * <b>possible</b> structure for your action model.
 * 
 * @author Nick Hawes
 * 
 */
public class PerfectActionModel implements ActionModel {

	@Override
	public GridPositionDistribution updateAfterMove(
			GridPositionDistribution _from, Heading _heading) {

		// Create the new distribution that will result from applying the action
		// model
		GridPositionDistribution to = new GridPositionDistribution(_from);

		// Move the probability in the correct direction for the action
		if (_heading == Heading.PLUS_X) {
			movePlusX(_from, to);
		} else if (_heading == Heading.PLUS_Y) {
			movePlusY(_from, to);
		} else if (_heading == Heading.MINUS_X) {
			moveMinusX(_from, to);
		} else if (_heading == Heading.MINUS_Y) {
			moveMinusY(_from, to);
		}

		return to;
	}

	/**
	 * Move probabilities from _from one cell in the plus x direction into _to
	 * 
	 * @param _from
	 * @param _to
	 */
	private void movePlusX(GridPositionDistribution _from,
			GridPositionDistribution _to) {

		// iterate through points updating as appropriate
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {

				// make sure to respect obstructed grid points
				if (!_to.isObstructed(x, y)) {					

					// position after move
					int toX = x;
					int toY = y;
					
					int fromX;
					int fromY;
					
					if(_to.isValidGridPosition(x-1, y)) {
						fromX = x-1;
						fromY = y;
						
						float fromProb = _from.getProbability(fromX, fromY);
					
						//if it is possible to move, do so, otherwise coordinates do not change
						
						if(_to.isValidTransition(fromX, fromY, toX, toY)) {
							_to.setProbability(toX, toY, fromProb);
						} else {
							_to.setProbability(fromX, fromY, 0);
							_to.setProbability(toX, toY, 0);
						}
					} else {
						_to.setProbability(toX, toY, 0);
					}

				}
			}
		}
		_to.normalise();
	}

	/**
	 * Move probabilities from _from one cell in the plus y direction into _to
	 * 
	 * @param _from
	 * @param _to
	 */
	private void movePlusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {
		
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
			
				if (!_to.isObstructed(x, y)) {					
					
					// position after move
					int toX = x;
					int toY = y;
					
					int fromX;
					int fromY;
					
					if(_to.isValidGridPosition(x, y-1)) {
						fromX = x;
						fromY = y-1;
						
						float fromProb = _from.getProbability(fromX, fromY);
					
						//if it is possible to move, do so, otherwise coordinates do not change
						
						if(_to.isValidTransition(fromX, fromY, toX, toY)) {
							_to.setProbability(toX, toY, fromProb);
						} else {
							_to.setProbability(fromX, fromY, 0);
							_to.setProbability(toX, toY, 0);
						}
					} else {
						_to.setProbability(toX, toY, 0);
					}
					
				}
				
			}
		}
		_to.normalise();
	}
	
	/**
	 * Move probabilities from _from one cell in the minus x direction into _to
	 * 
	 * @param _from
	 * @param _to
	 */
	private void moveMinusX(GridPositionDistribution _from,
			GridPositionDistribution _to) {
		
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
			
				if (!_to.isObstructed(x, y)) {					
					
					// position after move
					int toX = x;
					int toY = y;
					
					int fromX;
					int fromY;
					
					if(_to.isValidGridPosition(x+1, y)) {
						fromX = x+1;
						fromY = y;
						
						float fromProb = _from.getProbability(fromX, fromY);
					
						//if it is possible to move, do so, otherwise coordinates do not change
						
						if(_to.isValidTransition(fromX, fromY, toX, toY)) {
							_to.setProbability(toX, toY, fromProb);
						} else {
							_to.setProbability(fromX, fromY, 0);
							_to.setProbability(toX, toY, 0);
						}
					} else {
						_to.setProbability(toX, toY, 0);
					}			
					
				}
				
			}
		}
		_to.normalise();
	}
	
	/**
	 * Move probabilities from _from one cell in the minus y direction into _to
	 * 
	 * @param _from
	 * @param _to
	 */
	private void moveMinusY(GridPositionDistribution _from,
			GridPositionDistribution _to) {
		
		for (int y = 0; y < _to.getGridHeight(); y++) {

			for (int x = 0; x < _to.getGridWidth(); x++) {
			
				if (!_to.isObstructed(x, y)) {					
					
					// position after move
					int toX = x;
					int toY = y;
					
					int fromX;
					int fromY;
					
					if(_to.isValidGridPosition(x, y+1)) {
						fromX = x;
						fromY = y+1;
						
						float fromProb = _from.getProbability(fromX, fromY);
					
						//if it is possible to move, do so, otherwise coordinates do not change
						
						if(_to.isValidTransition(fromX, fromY, toX, toY)) {
							_to.setProbability(toX, toY, fromProb);
						} else {
							_to.setProbability(fromX, fromY, 0);
							_to.setProbability(toX, toY, 0);
						}
					} else {
						_to.setProbability(toX, toY, 0);
					}
					
				}
				
			}
		}
		_to.normalise();
	} 
}
