#include "stage.hh"
using namespace Stg;

const double cruisespeed = 0.4; 
const double avoidspeed = 0.05; 
const double avoidturn = 0.5;
const double minfrontdistance = 1.0; // 0.6  
const bool verbose = false;   // This parameter is simply for printing information on the screen
// const double stopdist = 0.3;  // this is the original value
const double stopdist = 0.6;
const int avoidduration = 10;

// Definition of the struct robot_t
typedef struct
{
  ModelPosition* pos;
  ModelRanger* laser;
  int avoidcount, randcount;
} robot_t;

// header for the LaserUpdate function
int LaserUpdate( Model* mod, robot_t* robot );

// header for the PositionUpdate function
int PositionUpdate( Model* mod, robot_t* robot );

// Stage calls this when the model starts up
// BRUNO: I do not really understand how this is working
extern "C" int Init( Model* mod, CtrlArgs* args )
{
  // local arguments
	/*  printf( "\nWander controller initialised with:\n"
			"\tworldfile string \"%s\"\n" 
			"\tcmdline string \"%s\"",
			args->worldfile.c_str(),
			args->cmdline.c_str() );
        */
	
  // declare an instance of the struct robot_t and call it robot (seriously this is sort of confusing)
  robot_t* robot = new robot_t;
 
  // set the fields avoidcount and randcount to 0
  robot->avoidcount = 0;
  robot->randcount = 0;
  
  // initialize the robot's field pos which is of type ModelPosition
  // I REALLY DO NOT GET WHAT IS GOING ON HERE
  robot->pos = (ModelPosition*)mod;

        // This is for printing some information on the screen
        // I do not understand this syntax with the callback and Model::CB_UPDATE ...
	if( verbose )
		robot->pos->AddCallback( Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot );

  // This I do understand, Subscribe is a function of the class ModelPosition or something like that
  robot->pos->Subscribe(); // starts the position updates

  // I do not understand the syntax of this command
  robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
  robot->laser->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserUpdate, robot );
  robot->laser->Subscribe(); // starts the ranger updates
   
  return 0; //ok
}


// inspect the ranger data and decide what to do
// THIS IS A BAD NAME FOR THIS FUNCTION: IT IS THE FUNCTION THAT GIVES COMMAND TO THE MOTORS, I.E., 
// IT DOES MUCH MORE THAN SIMPLY UPDATING THE LASER !!!!!!!
int LaserUpdate( Model* mod, robot_t* robot )
{
  // get the data
	const std::vector<meters_t>& scan = robot->laser->GetRanges();
  uint32_t sample_count = scan.size();
  if( sample_count < 1 )
    return 0;
  
  bool obstruction = false;
  bool stop = false;
	
  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++)
    {

		if( verbose ) printf( "%.3f ", scan[i] );

      // The condition implies that we are looking at the sensor in front of the robot
      if( (i > (sample_count/3)) 
			 && (i < (sample_count - (sample_count/3))) 
			 && scan[i] < minfrontdistance)
		  {
			 if( verbose ) puts( "  obstruction!" );
			 obstruction = true;
		  }
		
      if( scan[i] < stopdist )
		  {
			 if( verbose ) puts( "  stopping!" );
			 stop = true;
		  }

      // look at the reading on the left of the robot
      if( i > sample_count/2 )
				minleft = std::min( minleft, scan[i] );
      // look at the reading on the right of the robot
      else      
				minright = std::min( minright, scan[i] );
    }
  
  // Print some information 
  if( verbose ) 
	 {
		puts( "" );
		printf( "minleft %.3f \n", minleft );
		printf( "minright %.3f\n ", minright );
	 }
  
  // obstruction: something is too close in front of the robot
  // stop: something is too close from the robot from any of the scan[i] sensor reading
  // avoidcount: I am not too sure what is that. Originally it is set to 0
  if( obstruction || stop || (robot->avoidcount>0) )
    {
      // print some information
      if( verbose ) printf( "Avoid %d\n", robot->avoidcount );
      
      // If stop is true then we set the XSpeed to 0 otherwise we set it to avoidspeed	  		
      robot->pos->SetXSpeed( stop ? 0.0 : avoidspeed );      
      
      /* once we start avoiding, select a turn direction and stick
	 with it for a few iterations */
      if( robot->avoidcount < 1 )
        {
                         // print some information
			 if( verbose ) puts( "Avoid START" );
          robot->avoidcount = random() % avoidduration + avoidduration;
			 
                         // if the closest reading is coming from the left of the robot
			 if( minleft < minright  )
				{
				  robot->pos->SetTurnSpeed( -avoidturn );
				  if( verbose ) printf( "turning right %.2f\n", -avoidturn );
				}
			 else
				{
				  robot->pos->SetTurnSpeed( +avoidturn );
				  if( verbose ) printf( "turning left %2f\n", +avoidturn );
				}
        }
		
      robot->avoidcount--;
    }
  else
    {
      if( verbose ) puts( "Cruise" );

      robot->avoidcount = 0;
      robot->pos->SetXSpeed( cruisespeed );	  
			robot->pos->SetTurnSpeed(  0 );
    }

 //  if( robot->pos->Stalled() )
// 	 {
// 		robot->pos->SetSpeed( 0,0,0 );
// 		robot->pos->SetTurnSpeed( 0 );
// }
			
  return 0; // run again
}

int PositionUpdate( Model* mod, robot_t* robot )
{
  Pose pose = robot->pos->GetPose();

  printf( "Pose: [%.2f %.2f %.2f %.2f]\n",
	  pose.x, pose.y, pose.z, pose.a );

  return 0; // run again
}

