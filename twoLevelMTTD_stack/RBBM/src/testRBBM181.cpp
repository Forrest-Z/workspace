// $Id: testRBBM181.cpp 5925 2006-03-14 21:23:49Z tdelaet $
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
       << "test of RBBM" << endl
       << "==================================================" << endl;
  /***********************
   * PREPARE FILESTREAMS *
   **********************/
  ofstream fout;

  fout.open("RBBM181.out");
  const double p= 0.8;
  const double zmax = 8.0;  
  const double sigma_m = 0.05;
  const double pi3 = 0.2;  
  const double pi4 = 0.1;  
  const double radiusState = 0.4;  
  ColumnVector scannerPos(2);
  scannerPos(1)= 0.0;
  scannerPos(2)= 0.0;
  string environment_file("src/environment.txt");
  const int nbeams = 181;

  RBBM *rbbm = new RBBM(p,zmax,sigma_m,pi3,pi4,scannerPos,environment_file,nbeams,radiusState);
  cerr << "RBBM constructed" << endl;
  MeasurementModel<std::vector<ColumnVector> ,ColumnVector> *measModel = new MeasurementModel<std::vector<ColumnVector>,ColumnVector>(rbbm);
  cout << "Measurement model constructed " << endl;
    
  double step = 0.10; 
  ColumnVector state(2);
  state(1) = -0.0;
  state(2) = 2.5;
  //rbbm->ConditionalArgumentSet(0,state);
  //cout << "cond arg " <<  rbbm->ConditionalArgumentGet(0) << endl;

  //Get environment
  //Read environment from file
  ColumnVector environment(nbeams);
  ifstream indata; // indata is like cin
  double num; // variable for input value

  indata.open("cpf/environment.txt"); // opens the file
  if(!indata)  // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;

  indata >> num; // first one is timestamp
  indata >> num;
  int counter = 1;
  while ( !indata.eof()  && counter <= nbeams) //keep reading until end-of-file
  { 
      environment(counter) = num;
      indata >> num; // sets EOF flag if no value found
      counter++;
  }
  indata.close();

  vector<ColumnVector> measurement(2);
  ColumnVector angles(nbeams);
  for (int beam=0; beam<nbeams;beam++)
  {
    angles(beam+1)=(double)beam/((double)(nbeams-1)) * M_PI;
  }

  // set measurement equal to measurement with no targets
  measurement[0] = environment;
  measurement[1] = angles;
  // calculate which beam(s) are affected by the _state
  // TODO: now only one beam is considered affected

   /*
  // with full scan
  // For circular state you should here again calculate intersection of proposed
  // state and the beams => expected measurements = calculate probabilities with
  // that
  for (int beam=1; beam<=nbeams;beam++)
  {
       //cout << "expected measurement " << environment(beam) << endl;
       for (double z=0.0; z<=zmax+0.5; z+=step)   
       {
           measurement[0] = environment;
           //_measurement(beam) = sqrt( pow(_relPos(1),2.0) + pow(_relPos(2),2.0) );
           measurement[0](beam) = z;
           //cout << "z " << z << endl;
           //fout << beam << " " << z << " " << rbbm->ProbabilityGet(measurement) << endl;
           fout << beam << " " << z << " " << measModel->ProbabilityGet(measurement,state) << endl;
       }
  }
  */

  // with only one measurement
  measurement[0].resize(1);
  measurement[1].resize(1);
  for (int beam=0; beam<nbeams;beam++)
  {
       //cout << "expected measurement " << environment(beam) << endl;
       for (double z=0.0; z<=zmax+0.5; z+=step)   
       {
           //_measurement(beam) = sqrt( pow(_relPos(1),2.0) + pow(_relPos(2),2.0) );
           measurement[0] = z;
           measurement[1]= angles(beam+1);
           //cout << "measurement[0] " << measurement[0] << endl; 
           //cout << "measurement[1] " << measurement[1] << endl; 
           //fout << beam << " " << z << " " << rbbm->ProbabilityGet(measurement) << endl;
           fout << beam << " " << z << " " << measModel->ProbabilityGet(measurement,state) << endl;
       }
  }


  /****************************
   * CLOSE FILESTREAMS
   ***************************/
  fout.close();
  delete rbbm;
  delete measModel;
  return 0;
}
