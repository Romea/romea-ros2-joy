#ifndef _romea_JoystickMapping_hpp_
#define _romea_JoystickMapping_hpp_

//ros
#include <romea_common_utils/params/node_parameters.hpp>
#include <sensor_msgs/msg/joy.hpp>

//std
#include <map>


namespace romea
{


class JoystickMapping
{

public :

  using StringToStringMap = std::map<std::string,std::string>;
  using StringToId = std::map<std::string,int>;

public :

  JoystickMapping(const StringToStringMap & name_remappings,
                  const bool & keep_only_remapped_ones);

  StringToId get(const StringToId &id_mappings) const;

private :

  StringToStringMap name_remappings_;
  bool keep_only_remapped_ones_;
};
}

#endif
