/*****************************************************************
 *
 *
 *****************************************************************/

/*! \file LinearAssignmentProblem.h
 \brief Header file of the LinearAssignmentProblem class.
 */
#ifndef ASSIGNMENTPROBLEMTEST_H_
#define ASSIGNMENTPROBLEMTEST_H_

#include <mht/assignment_problem/AssignmentProblem.h>
#include <lap/CompleteCostMatrix.h>

#define DEBUG_ASSIGNMENTPROBLEMTEST 0

using namespace lap;
using namespace mht;
namespace lap {

/**
 * The LinearAssignmentProblem class implements a test for constructing an
 * assignment matrix
 *
 * @author: Tinne De Laet
 */
class LinearAssignmentProblem
{

public:

	/** Constructor. */
	LinearAssignmentProblem();
	/** Destructor. */
	virtual ~LinearAssignmentProblem()
	{
	}
	/** Create the cost matrix. See AssignmentProblem. */
	void createGlobalCostMatrix(int numTargets, int numMeasurements, const std::vector<std::vector<double> >& matchProbabilities,
			CompleteCostMatrix& costMatrix);

protected:

	/// Costs of a detection 
	int _detectionCosts;
	/// Costs of a occlusion 
	int _occlusionCosts;
	/// Costs of a deletion 
	int _deletionCosts;
	/// Costs of a new Track
	int _newTrackCosts;
	/// Costs of a false alarm
	int _falseAlarmCosts;

private:

};

}

#endif /* ASSIGNMENTPROBLEMTEST_H_ */
