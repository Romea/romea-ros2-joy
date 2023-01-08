# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from romea_common_bringup import MetaDescription


class JoystickMetaDescription:
    def __init__(self, meta_description_filename):
        self.meta_description = MetaDescription("joystick", meta_description_filename)

    def get_name(self):
        return self.meta_description.get("name")

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_device(self):
        return self.meta_description.get("device", "driver")

    def get_driver_autorepeat_rate(self):
        return self.meta_description.get("autorepeat_rate", "driver")

    def get_driver_dead_zone(self):
        return self.meta_description.get("deadzone", "driver")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    # def get_model(self):
    #     return self.meta_description.get("model", "configuration")
