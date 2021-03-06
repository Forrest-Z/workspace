//#line 2 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
// *********************************************************
// 
// File autogenerated for the hokuyo_node package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __hokuyo_node__HOKUYOCONFIG_H__
#define __hokuyo_node__HOKUYOCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/config_init_mutex.h>

namespace hokuyo_node
{
  class HokuyoConfigStatics;
  
  class HokuyoConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(HokuyoConfig &config, const HokuyoConfig &max, const HokuyoConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const HokuyoConfig &config1, const HokuyoConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, HokuyoConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const HokuyoConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, HokuyoConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const HokuyoConfig &config) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T HokuyoConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (HokuyoConfig::* field);

      virtual void clamp(HokuyoConfig &config, const HokuyoConfig &max, const HokuyoConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const HokuyoConfig &config1, const HokuyoConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, HokuyoConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const HokuyoConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, HokuyoConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const HokuyoConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }
    };

//#line 46 "../cfg/Hokuyo.cfg"
      double min_ang;
//#line 47 "../cfg/Hokuyo.cfg"
      double max_ang;
//#line 48 "../cfg/Hokuyo.cfg"
      bool intensity;
//#line 49 "../cfg/Hokuyo.cfg"
      int cluster;
//#line 50 "../cfg/Hokuyo.cfg"
      int skip;
//#line 51 "../cfg/Hokuyo.cfg"
      std::string port;
//#line 52 "../cfg/Hokuyo.cfg"
      bool calibrate_time;
//#line 53 "../cfg/Hokuyo.cfg"
      std::string frame_id;
//#line 54 "../cfg/Hokuyo.cfg"
      double time_offset;
//#line 55 "../cfg/Hokuyo.cfg"
      bool allow_unsafe_settings;
//#line 138 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        if ((*i)->fromMessage(msg, *this))
          count++;
      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("HokuyoConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      __toMessage__(msg, __param_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->fromServer(nh, *this);
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const HokuyoConfig &__max__ = __getMax__();
      const HokuyoConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const HokuyoConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const HokuyoConfig &__getDefault__();
    static const HokuyoConfig &__getMax__();
    static const HokuyoConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    
  private:
    static const HokuyoConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void HokuyoConfig::ParamDescription<std::string>::clamp(HokuyoConfig &config, const HokuyoConfig &max, const HokuyoConfig &min) const
  {
    return;
  }

  class HokuyoConfigStatics
  {
    friend class HokuyoConfig;
    
    HokuyoConfigStatics()
    {
//#line 46 "../cfg/Hokuyo.cfg"
      __min__.min_ang = -3.14159265359;
//#line 46 "../cfg/Hokuyo.cfg"
      __max__.min_ang = 3.14159265359;
//#line 46 "../cfg/Hokuyo.cfg"
      __default__.min_ang = -1.57079632679;
//#line 46 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<double>("min_ang", "double", 1, "The angle of the first range measurement. The unit depends on ~ang_radians.", "", &HokuyoConfig::min_ang)));
//#line 47 "../cfg/Hokuyo.cfg"
      __min__.max_ang = -3.14159265359;
//#line 47 "../cfg/Hokuyo.cfg"
      __max__.max_ang = 3.14159265359;
//#line 47 "../cfg/Hokuyo.cfg"
      __default__.max_ang = 1.57079632679;
//#line 47 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<double>("max_ang", "double", 1, "The angle of the first range measurement. The unit depends on ~ang_radians.", "", &HokuyoConfig::max_ang)));
//#line 48 "../cfg/Hokuyo.cfg"
      __min__.intensity = 0;
//#line 48 "../cfg/Hokuyo.cfg"
      __max__.intensity = 1;
//#line 48 "../cfg/Hokuyo.cfg"
      __default__.intensity = 0;
//#line 48 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<bool>("intensity", "bool", 1, "Whether or not the hokuyo returns intensity values.", "", &HokuyoConfig::intensity)));
//#line 49 "../cfg/Hokuyo.cfg"
      __min__.cluster = 0;
//#line 49 "../cfg/Hokuyo.cfg"
      __max__.cluster = 99;
//#line 49 "../cfg/Hokuyo.cfg"
      __default__.cluster = 1;
//#line 49 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<int>("cluster", "int", 1, "The number of adjacent range measurements to cluster into a single reading", "", &HokuyoConfig::cluster)));
//#line 50 "../cfg/Hokuyo.cfg"
      __min__.skip = 0;
//#line 50 "../cfg/Hokuyo.cfg"
      __max__.skip = 9;
//#line 50 "../cfg/Hokuyo.cfg"
      __default__.skip = 0;
//#line 50 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<int>("skip", "int", 1, "The number of scans to skip between each measured scan", "", &HokuyoConfig::skip)));
//#line 51 "../cfg/Hokuyo.cfg"
      __min__.port = "";
//#line 51 "../cfg/Hokuyo.cfg"
      __max__.port = "";
//#line 51 "../cfg/Hokuyo.cfg"
      __default__.port = "/dev/ttyACM0";
//#line 51 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<std::string>("port", "str", 3, "The serial port where the hokuyo device can be found", "", &HokuyoConfig::port)));
//#line 52 "../cfg/Hokuyo.cfg"
      __min__.calibrate_time = 0;
//#line 52 "../cfg/Hokuyo.cfg"
      __max__.calibrate_time = 1;
//#line 52 "../cfg/Hokuyo.cfg"
      __default__.calibrate_time = 1;
//#line 52 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<bool>("calibrate_time", "bool", 3, "Whether the node should calibrate the hokuyo's time offset", "", &HokuyoConfig::calibrate_time)));
//#line 53 "../cfg/Hokuyo.cfg"
      __min__.frame_id = "";
//#line 53 "../cfg/Hokuyo.cfg"
      __max__.frame_id = "";
//#line 53 "../cfg/Hokuyo.cfg"
      __default__.frame_id = "laser";
//#line 53 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<std::string>("frame_id", "str", 0, "The frame in which laser scans will be returned", "", &HokuyoConfig::frame_id)));
//#line 54 "../cfg/Hokuyo.cfg"
      __min__.time_offset = -0.25;
//#line 54 "../cfg/Hokuyo.cfg"
      __max__.time_offset = 0.25;
//#line 54 "../cfg/Hokuyo.cfg"
      __default__.time_offset = 0.0;
//#line 54 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<double>("time_offset", "double", 0, "An offet to add to the timestamp before publication of a scan", "", &HokuyoConfig::time_offset)));
//#line 55 "../cfg/Hokuyo.cfg"
      __min__.allow_unsafe_settings = 0;
//#line 55 "../cfg/Hokuyo.cfg"
      __max__.allow_unsafe_settings = 1;
//#line 55 "../cfg/Hokuyo.cfg"
      __default__.allow_unsafe_settings = 0;
//#line 55 "../cfg/Hokuyo.cfg"
      __param_descriptions__.push_back(HokuyoConfig::AbstractParamDescriptionConstPtr(new HokuyoConfig::ParamDescription<bool>("allow_unsafe_settings", "bool", 3, "Turn this on if you wish to use the UTM-30LX with an unsafe angular range. Turning this option on may cause occasional crashes or bad data. This option is a tempory workaround that will hopefully be removed in an upcoming driver version.", "", &HokuyoConfig::allow_unsafe_settings)));
//#line 239 "/opt/ros/electric/stacks/driver_common/dynamic_reconfigure/templates/ConfigType.h"
    
      for (std::vector<HokuyoConfig::AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        __description_message__.parameters.push_back(**i);
      __max__.__toMessage__(__description_message__.max, __param_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__); 
    }
    std::vector<HokuyoConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    HokuyoConfig __max__;
    HokuyoConfig __min__;
    HokuyoConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;
    static const HokuyoConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static HokuyoConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &HokuyoConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const HokuyoConfig &HokuyoConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const HokuyoConfig &HokuyoConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const HokuyoConfig &HokuyoConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<HokuyoConfig::AbstractParamDescriptionConstPtr> &HokuyoConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const HokuyoConfigStatics *HokuyoConfig::__get_statics__()
  {
    const static HokuyoConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = HokuyoConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __HOKUYORECONFIGURATOR_H__
