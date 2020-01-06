from teleop.utils.registry_meta import teleop_registry


def make(config_data, section_name):
    class_name = config_data.get(section_name, 'class_name')
    try:
        return teleop_registry[class_name](config_data, section_name)
    except KeyError:
        print("teleop_registry does not contain: {}".format(class_name))

