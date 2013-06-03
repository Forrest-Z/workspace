#include "linearAssignmentProblem.h"
#include <mht/handler/AssignmentHandler.h>

using namespace std;
using namespace lap;
using namespace mht;
namespace lap {

/**
 * Constructor.
 */
LinearAssignmentProblem::LinearAssignmentProblem() 
{
	// calculate costs
	{
		// free tracks
		double pDet = 0.8;
		double pOcc = 0.17;
		double pDel = 0.03;
		double pSum = pDet + pOcc + pDel;
		assert(pSum> 0);
		if (pSum < 0.9999 || pSum > 1.0001) {
			warning(
					"LinearAssignmentProblem: Probabilities for Tracks do NOT sum to 1! Normalization done!");
		}

		pSum = 1.0 / pSum;
		_detectionCosts
				= (int) (-1 * DOUBLE_MULTIPLIER * log(pDet * pSum));
		_occlusionCosts
				= (int) (-1 * DOUBLE_MULTIPLIER * log(pOcc * pSum));
		_deletionCosts= (int) (-1 * DOUBLE_MULTIPLIER * log(pDel * pSum));
	}
	{
		// new Tracks, false alarms
		double pNew = 0.01;
		double pFal = 0.01;
		_newTrackCosts = (int) (-1 * DOUBLE_MULTIPLIER * log(pNew));
		_falseAlarmCosts = (int) (-1 * DOUBLE_MULTIPLIER * log(pFal));
	}
}

 void LinearAssignmentProblem::createGlobalCostMatrix(int numTargets, int numMeasurements, const
        std::vector<vector<double> >& matchProbabilities,
		CompleteCostMatrix& costMatrix)
{
    // pre: costMatrix should be of correct size
    if(! (matchProbabilities.size() == numTargets && (numTargets==0 || matchProbabilities[0].size()==numMeasurements) ) )
        cout << "ERROR: matchProbabilities of incorrect size" << endl;
	unsigned int nTracks = numTargets;
	unsigned int nObservations = numMeasurements;
	unsigned int nTrackLabels = nTracks + 2 * nObservations; // Tracks + new Tracks + false alarms
	unsigned int nObservationLabels = nObservations + 2 * nTracks; // Observations + occlusions + deletions
    unsigned int dim = max(nTrackLabels,nObservationLabels);
    if(! (costMatrix.getRows() == dim && costMatrix.getCols() == dim ))
        cout << "ERROR: costMatrix of incorrect size" << endl;

	// ====================================================================================================
	// set detection costs == _detectionCosts[Free | Approved] + probability of assignment
	for (unsigned int tl = 0; tl < nTracks; ++tl)  // loop over tracks
    {
	    for (unsigned int  ol = 0; ol <  nObservations;  ++ol)  // loop over observations
        {
            costMatrix(tl,ol) = (int) (-1 * DOUBLE_MULTIPLIER * log(matchProbabilities[tl][ol]) ) + _detectionCosts;
        }
    }
        
	// ====================================================================================================
	// set occlusion costs: each Track can be assigned to one "occlusion" label
	for (unsigned int tl = 0, ol = nObservations; tl < nTracks && ol
			< nObservations + nTracks; ++tl, ++ol) {
            costMatrix(tl,ol) = _occlusionCosts;
	}
	// ====================================================================================================
	// set deletion costs: each Track can be assigned to one "deletion" label
	for (unsigned int tl = 0, ol = nObservations + nTracks; tl < nTracks && ol
			< nObservationLabels; ++tl, ++ol) {
            costMatrix(tl,ol) = _deletionCosts;
	}
	// ====================================================================================================
	// set newTrack costs: each Observation can be assigned to one "newTrack" label
	for (unsigned int tl = nTracks, ol = 0; tl < nTracks + nObservations && ol
			< nObservations; ++tl, ++ol) {
            costMatrix(tl,ol) = _newTrackCosts;
	}
	// ====================================================================================================
	// set false alarm costs: each Observation can be assigned to one "false alarm" label
	for (unsigned int tl = nTracks + nObservations, ol = 0; tl < nTrackLabels
			&& ol < nObservations; ++tl, ++ol) {
            costMatrix(tl,ol) = _falseAlarmCosts;
	}
	// this part of the matrix is not read by any assignment, so it does not need to be filled
	// ====================================================================================================
	// occlusions and deletions can be assigned to new Track and false alarms
	for (unsigned int tl = nTracks; tl < nTrackLabels; ++tl) {
		for (unsigned int ol = nObservations; ol < nObservationLabels; ++ol) {
            costMatrix(tl,ol) = 0;
		}
	}
    // if nTracks is not equal to nObsevations => fill in values added to make
    // matrix square
	// ====================================================================================================
	for (unsigned int tl = nTrackLabels; tl < dim ; ++tl) {
		for (unsigned int ol = nObservations; ol < dim; ++ol) {
            costMatrix(tl,ol) = 0;
		}
	}
    for (unsigned int ol = nObservationLabels; ol < dim; ++ol) {
    	for (unsigned int tl = nTracks; tl < dim ; ++tl) {
            costMatrix(tl,ol) = 0;
        }
	}
}

}
