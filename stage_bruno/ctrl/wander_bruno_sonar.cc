#include "stage.hh"
using namespace Stg;

const double cruisespeed = 0.4; 
const double avoidspeed = 0.05; 
const double avoidturn = 0.5;
const double minfrontdistance = 1.0; // 0.6  
const bool verbose = false;   // This parameter is simply for printing information on the screen
// const double stopdist = 0.3;  // this is the original value
const double stopdist = 0.45;
// const int avoidduration = 10;   // this is the original value
const int avoidduration = 15;

// Definition of the struct robot_t
typedef struct
{
  ModelPosition* pos;
  ModelRanger* ranger;
  int avoidcount, randcount;
} robot_t;

// header for the LaserUpdate function
int LaserUpdate( ModelRanger* mod, robot_t* robot );

// header for the PositionUpdate function
int PositionUpdate( Model* mod, robot_t* robot );

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
  robot_t* robot = new robot_t;

  // set the fields avoidcount and randcount to 0
  robot->avoidcount = 0;
  robot->randcount = 0;
  robot->pos = (ModelPosition*)mod;

  // subscribe to the ranger, which we use for navigating
  robot->ranger = (ModelRanger*)mod->GetUnusedModelOfType( "ranger" );
  assert( robot->ranger );
  
  // ask Stage to call into our ranger update function
  robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)LaserUpdate, robot );
  
  robot->ranger->Subscribe();
  robot->pos->Subscribe();

  return 0; //ok
}


// inspect the ranger data and decide what to do
// THIS IS A BAD NAME FOR THIS FUNCTION: IT IS THE FUNCTION THAT GIVES COMMAND TO THE MOTORS, I.E., 
// IT DOES MUCH MORE THAN SIMPLY UPDATING THE LASER !!!!!!!
int LaserUpdate( ModelRanger* rgr, robot_t* robot )
{

  // getchar();

  // get the data
	// const std::vector<meters_t>& scan = robot->laser->GetRanges();   // previously

        const std::vector<ModelRanger::Sensor>& sensors = rgr->GetSensors();


  //uint32_t sample_count = scan.size();
  //if( sample_count < 1 )
  //  return 0;
  
  bool obstruction = false;
  bool stop = false;
	
  // find the closest distance to the left and right and check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  int sample_count = 8;

  // The number of sonar sensors used for now is 8
  for (uint32_t i = 0; i < sample_count; i++)
    {

		if( verbose ) printf( "%.3f ", sensors[i].ranges[0] );

      // The condition implies that we are looking at the sensor in front of the robot     
      if( (i == 3 || i == 4) && sensors[i].ranges[0] < minfrontdistance)      
		  {
			 if( verbose ) puts( "  obstruction!" );
			 obstruction = true;
		  }
		
      if( sensors[i].ranges[0] < stopdist )
		  {
			 if( verbose ) puts( "  stopping!" );
			 stop = true;
		  }

      // look at the reading on the left of the robot
      if( i == 0 || i == 1 || i == 2 )
				minleft = std::min( minleft, sensors[i].ranges[0] );
      // look at the reading on the right of the robot
      else if(i == 5 || i == 6 || i == 7)      
				minright = std::min( minright, sensors[i].ranges[0] );
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

