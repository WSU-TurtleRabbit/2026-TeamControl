
from pathlib import Path

import ast
import yaml

try:
    from yaml import CLoader as Loader
except ImportError as e:
    from yaml import Loader


          
class Config():
    def __init__(self,config_filename:str="ipconfig.yaml"):

        path = Path(__file__).resolve()
        # use with the auto open and close
        with open(path.parent / config_filename, "r") as file:
            # read and sets the robot addr and base ID
            self.set_config(yaml.load(file, Loader))
            
    def set_config(self,raw):
        self.blue = raw["blue"]
        self.yellow = raw["yellow"]
        self.grSim_addr = (raw["grSim"]["ip"], raw["grSim"]["port"])
        self.vision = raw["vision"]["multicast-group"], raw["vision"]["port"] 
        self.game_controller = raw["gc"]["multicast-group"], raw["gc"]["port"]
        
        net = raw.get("network", {})
        self.robot_ip = net.get("robot_ip", "192.168.1.2")
        self.vision_ip = net.get("vision_ip", "192.168.1.2")
        self.use_grSim_vision = raw["use_grSim_vision"]
        self.us_yellow = raw["us_yellow"]
        self.us_positive = raw["us_positive"]
        self.send_to_grSim = raw["send_to_grSim"]
        self.team_name = raw.get("team_name", "TurtleRabbit")
   
    

if __name__ == "__main__":
    server_config = Config()
    print(server_config.blue)