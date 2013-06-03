// $Id: testRBBM.cpp 5925 2006-03-14 21:23:49Z tdelaet $
// Copyright (C) 2009
//                    Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.


#include "RBBM.h"//added
#include <bfl/model/analyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/gaussian.h>

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/wrappers/matrix/vector_wrapper.h>

#include <iostream>
#include <fstream>
#include <string>


/* The purpose of this program is to test the RBBM
*/

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

int main(int argc, char** argv)
{
  cerr << "==================================================" << endl
       << "test of RBBM Meas Model" << endl
       << "==================================================" << endl;

  /***********************
   * PREPARE FILESTREAMS *
   **********************/
  ofstream fout;

  fout.open("RBBM.out");
  const double p= 0.65;
  const double zmax = 8.0;  
  const double sigma_m = 0.25;
  const double pi3 = 0.2;  
  const double pi4 = 0.1;  
  ColumnVector scannerPos(2);
  scannerPos(1)= 0.0;
  scannerPos(2)= 0.0;
  string environment_file("src/test1.txt");
  const int nbeams = 1;

  RBBM *rbbm = new RBBM(p,zmax,sigma_m,pi3,pi4,scannerPos,environment_file,nbeams);
  cout << "RBBM constructed" << endl;
  MeasurementModel<std::vector<ColumnVector> ,ColumnVector> *measModel = new MeasurementModel<std::vector<ColumnVector>,ColumnVector>(rbbm);
  cout << "MeasurementModel constructed" << endl;
    
  double step = 0.01; 
  ColumnVector state(2);
  state(1) = 5.0;
  state(2) = 0.0;

  vector<ColumnVector> measurement(2);
  measurement[0] = ColumnVector(nbeams);
  measurement[1] = ColumnVector(nbeams);
  // only angle 0
  measurement[1](1)=0.0;
  cout <<"measurement[0] set" << endl;
  for(measurement[0](1)=0.0;measurement[0](1)<zmax+0.5;measurement[0](1)+=step)
  {
    cout << "measurement " << measurement[0](1) << " " << measurement[1](1) <<endl;
    fout << measurement[0](1) << " " << measurement[1](1) << " " << (double)measModel->ProbabilityGet(measurement,state) << endl;
  }


  /****************************
   * CLOSE FILESTREAMS
   ***************************/
  fout.close();
  delete rbbm;
  delete measModel;
  return 0;
}
