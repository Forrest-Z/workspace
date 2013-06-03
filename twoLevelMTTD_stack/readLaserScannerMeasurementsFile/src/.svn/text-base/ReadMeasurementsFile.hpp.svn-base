// Copyright  (C)  2009  Wilm Decre <Wilm dot Decre at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Tinne De Laet <Tinne dot DeLaet at mech dot kuleuven dot be>

// Author: Tinne De Laet, Wilm Decre
// Maintainer: Tinne De Laet, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef __READMEASUREMENTSFILE__
#define __READMEASUREMENTSFILE__

#include <rtt/TaskContext.hpp>
#include <rtt/Logger.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Method.hpp>
#include <rtt/Command.hpp>
#include <rtt/Event.hpp>
#include <rtt/Ports.hpp>
#include <rtt/TimeService.hpp>

#include <ocl/OCL.hpp>


#include <fstream>
using std::ifstream;

using namespace std;
using namespace RTT;
using namespace BFL;
using namespace Orocos;

namespace OCL
{

    /**
        ReadMeasurementsFile reads laserscanner measurements from a file. 
        The file should have the following columns timestamp | LaserDistance | LaserAngle 
     */
    class ReadMeasurementsFile
        : public TaskContext
    {
    protected:
        /*PROPERTIES*/
        /// The name of the text file containing the measurements
        Property<std::string>          _measurements_file;
        /// The number of beams of the laser scanner
        Property<int>                  _nbeams;

        /*METHODS*/
        /// Read a measurement from the file
        Method<void(void) >            _readMeasurement;

        /*DATA PORTS*/
        /// Write Data port for the distances of the read laser scanner measurement
        WriteDataPort<std::vector<double> > _distances;
        /// Write Data port for the angles of the read laser scanner measurement
        WriteDataPort<std::vector<double> > _angles;

    private:
        /// Helper variable to store the distances of the read laser scanner measurement
        std::vector<double>         _distances_local; 
        /// Helper variable to store the angles of the read laser scanner measurement
        std::vector<double>         _angles_local; 
        /// File stream to read the measurement
        ifstream                    _measurementStream;
        /// Helper variable to check the timing
        RTT::TimeService::ticks     time_begin;
        /// Helper variable to check the timing
        double                      time_passed;

        /*
        * Read a measurement from the text file
        */
        void                        readMeasurement(); 

    public:
        /*
        * Constructor
        * @param name the name of the component
        */
        ReadMeasurementsFile(std::string name);
        /*
        * Destructor
        */
        ~ReadMeasurementsFile();
        
        bool        configureHook();
        bool        startHook();
        void        updateHook(){};
        void        stopHook(){};
        void        cleanUpHook(){};
        
    };
}
#endif // __READMEASUREMENTSFILE__
