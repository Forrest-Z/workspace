#include <lap/hungarian.h>
#include <lap/LAPJV.h>
#include <lap/CompleteCostMatrix.h>
#include <lap/LAPJVShared.h>
#include <lap/LAPJVTemplate.h>
#include <lap/LapGenerator.h>
#include <base/util/Timer.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <map>
#include "linearAssignmentProblem.h"

int main()
{
    using namespace lap; 
    using namespace std; 

    bool print = true;

    // Create costMatrix
    int numTargets = 3;
    int numMeasurements = 4;

    const unsigned int dimR = numTargets + 2* numMeasurements;
    const unsigned int dimC = numMeasurements + 2* numTargets;
    unsigned int dim = max(dimR, dimC);
    CompleteCostMatrix costmatrix(dim, dim);
    costmatrix.setConstant(ASSIGNMENTSOLVER_INFINITY);

    // for testing: fill out p(meausement | target)
    vector<vector<double> > matchProbabilities(numTargets);
    matchProbabilities.assign(numTargets,vector<double>(numMeasurements));

    matchProbabilities[0][0] = 0.01;    
    matchProbabilities[0][1] = 0.55;    
    matchProbabilities[0][2] = 0.05;    
    matchProbabilities[0][3] = 0.02;    
    matchProbabilities[1][0] = 0.03;    
    matchProbabilities[1][1] = 0.04;    
    matchProbabilities[1][2] = 0.54;    
    matchProbabilities[1][3] = 0.02;    
    matchProbabilities[2][0] = 0.03;    
    matchProbabilities[2][1] = 0.04;    
    matchProbabilities[2][2] = 0.04;    
    matchProbabilities[2][3] = 0.02;    
    
    // create AssignmentProblemTest: will create full costmatrix from
    // matchProbabilities
    LinearAssignmentProblem* assignmentProblemTest = new LinearAssignmentProblem();
    assignmentProblemTest->createGlobalCostMatrix(numTargets,numMeasurements,matchProbabilities,costmatrix);

    // Solve assignmentproblem
    LAPJVShared lapjv(dim, &costmatrix);
    lapjv.run();
    const int* colSolution = lapjv.getColumnSolution();
    for (unsigned int col = 0; col < dim; ++col) {
       // check validity
       std::cout << "solution for feature " << col << ": object " << colSolution[col] << "with cost " << costmatrix(colSolution[col], col)<<std::endl; 
       if (costmatrix(colSolution[col], col) < ASSIGNMENTSOLVER_INFINITY) {
           // valid assignment
            cout << "col " << col << " results in valid assignment" << endl;
       }
    }
	if (print) {
        cout <<"cost matrix " << endl;
		lapjv.printCostMatrix();
        cout <<"assignment matrix " << endl;
		lapjv.printAssignmentMatrix();
	}

    delete assignmentProblemTest;
}
