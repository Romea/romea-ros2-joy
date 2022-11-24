#include "romea_joystick_utils/joystick_remapping.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickRemapping::JoystickRemapping(const StringToStringMap &name_remappings,
                                     const bool & keep_only_remapped_ones):
  inverted_name_remappings_(),
  keep_only_remapped_ones_(keep_only_remapped_ones),
  status_()
{
  for (const auto& it : name_remappings)
  {
    inverted_name_remappings_.insert(make_pair(it.second, it.first));
    status_.insert(make_pair(it.first, false));
  }
}


//-----------------------------------------------------------------------------
JoystickRemapping::StringToId JoystickRemapping::apply(const StringToId &id_mappings) const
{
  std::map<std::string, int> result;

  for (const auto & [name, id] : id_mappings)
  {
    if (auto it = inverted_name_remappings_.find(name);
        it != inverted_name_remappings_.end())
    {
      result[it->second] = id;
      status_[it->second] = true;
    } else if (!keep_only_remapped_ones_) {
      result[name] = id;
    }
  }

  return result;
}


//-----------------------------------------------------------------------------
const JoystickRemapping::Status & JoystickRemapping::get_status() const
{
  return status_;
}

//-----------------------------------------------------------------------------
bool JoystickRemapping::is_complete() const
{
  for (const auto & [name, status] : status_)
  {
    if (!status)
    {
      return false;
    }
  }

  return true;
}

}  // namespace romea

