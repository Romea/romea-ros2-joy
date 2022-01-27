#include "romea_joy/joystick_mapping.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickMapping::JoystickMapping(const StringToStringMap &name_remappings,
                                 const bool & keep_only_remapped_ones):
  name_remappings_(name_remappings),
  keep_only_remapped_ones_(keep_only_remapped_ones)
{
}


//-----------------------------------------------------------------------------
JoystickMapping::StringToId JoystickMapping::get(const StringToId &id_mappings)const
{

  std::map<std::string,int> result;
  for(const auto & [name,id] : id_mappings)
  {
    if(auto it=name_remappings_.find(name);it!=name_remappings_.end())
    {
      result[it->second]=id;
    }
    else if(!keep_only_remapped_ones_)
    {
      result[name]=id;
    }
  }

  return result;
}

}

