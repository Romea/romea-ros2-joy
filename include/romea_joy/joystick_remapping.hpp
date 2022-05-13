#ifndef _romea_JoystickMapping_hpp_
#define _romea_JoystickMapping_hpp_

//ros
#include <romea_common_utils/params/node_parameters.hpp>
#include <sensor_msgs/msg/joy.hpp>

//std
#include <map>


namespace romea
{


class JoystickRemapping
{

public :


  using StringToStringMap = std::map<std::string,std::string>;
  using StringToId = std::map<std::string,int>;
  using Status = std::map<std::string,bool>;

public :

  JoystickRemapping(const StringToStringMap & name_remappings,
                    const bool & keep_only_remapped_ones);

  StringToId apply(const StringToId &id_mappings)const;

  const Status & get_status() const;

  bool is_complete() const;

private :

  StringToStringMap inverted_name_remappings_;
  bool keep_only_remapped_ones_;
  mutable Status status_;
};
}

#endif
