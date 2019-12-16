import shutil
import os
from shutil import copyfile

class PluginManager:
    def __init__(self):
        pass
    
    @staticmethod
    def get_workspace_dir():
        workspace_path = os.path.normpath(os.getcwd())
        workspace_dirs = []
        for path_dir in workspace_path.split(os.sep):
            if path_dir == "src":
                break
            workspace_dirs.append(path_dir)
        
        return os.sep.join(workspace_dirs)

    @staticmethod
    def validate_dir(dir_path):
        if not os.path.isdir(dir_path):
            raise FileNotFoundError("{dir_path} not exist".format)

    @staticmethod
    def get_devel_path(workspace_dir):
        devel_dirs =  workspace_dir.split(os.sep)
        devel_dirs.append("devel")
        devel_dirs.append("lib")
        
        return os.sep.join(devel_dirs)

    @staticmethod
    def get_gazebo_simulation_package_path(workspace_dir):
        simulation_package_dirs = workspace_dir.split(os.sep)
        simulation_package_dirs.append("src")
        simulation_package_dirs.append("gazebo_simulations")
        simulation_package_dirs.append("plugins")

        return os.sep.join(simulation_package_dirs)

    @staticmethod
    def update_plugins():
        workspace_dir = PluginManager.get_workspace_dir()
        devel_dir = PluginManager.get_devel_path(workspace_dir)
        gazebo_simulation_dir = PluginManager.get_gazebo_simulation_package_path(workspace_dir)
        PluginManager.validate_dir(devel_dir)
        PluginManager.validate_dir(gazebo_simulation_dir) 
        PluginManager.copy_plugins(devel_dir, gazebo_simulation_dir)

    @staticmethod
    def copy_plugin(plugin_path, gazebo_simulation_dir):
        copyfile(plugin_path, gazebo_simulation_dir)

    @staticmethod        
    def copy_plugins(devel_dir, gazebo_simulation_dir):

        for plugin in os.listdir(devel_dir):
            if plugin.endswith(".so"):
                source_path =  os.sep.join([devel_dir,plugin])
                destination_path = os.sep.join([gazebo_simulation_dir,plugin])
                print("{} copy to devel dir: {}". format(source_path, destination_path))
                PluginManager.copy_plugin(source_path, destination_path)


            