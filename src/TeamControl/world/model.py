'''
 World Model - Central Storage and access control
- apply this in multiprocessing or equivalent 
@author - Emma
'''

from TeamControl.SSL.vision.frame_list import FrameList
from TeamControl.SSL.vision.field import GeometryData,FieldSize
from TeamControl.SSL.vision.frame import Frame
from TeamControl.SSL.game_controller.common import Command,Stage,GameEventType,Team,PacketType, GameState
from TeamControl.SSL.game_controller.Message import RefereeMessage,TeamInfo

from multiprocessing import Queue,Manager
import numpy as np
import numpy.typing as npt
import logging
import time


log = logging.getLogger()
log.setLevel(logging.DEBUG)


class WorldModel:
    """
    World Model aka wm
    Description : 
        ...
    """

    def __init__(
        self,
        update_interval: int = 10,
        history: int = 60,
        use_sim: bool = True,
        us_yellow: bool = True,
        us_positive: bool = True,
    ):
        mgr = Manager()
        self._us_yellow = us_yellow
        self._us_positive = us_positive
        self.count = 0
        self.update_interval:int = update_interval
        self.use_sim:bool = use_sim 
        self.frame_list:FrameList[Frame] = FrameList(history=history)
        self.geometry:GeometryData = None
        self.field:FieldSize = None
        self._version = mgr.Value('i', 0)   # int counter
        self._state = None # current state from GC
        self.robot_active = 6 # robots active
        self.blf_location = None # ball left field location
        self.ball_placement_pos: tuple | None = None  # designated position for ball placement
        self._onboard_store = {}  # (is_yellow, robot_id) -> OnboardObservation
    
    def update_game_data(self,game_data):
        if game_data is None:
            return
        if isinstance(game_data,Command):
            self.ref_data.command = game_data

        elif isinstance(game_data, Stage):
            self.ref_data.stage = game_data

        elif isinstance(game_data, tuple):
            if isinstance(game_data[0], TeamInfo):
                self.ref_data.yellow = game_data[0]
                self.ref_data.blue = game_data[1]

    def update_team(self, us_yellow: bool, us_positive: bool):
        self._us_yellow = us_yellow
        self._us_positive = us_positive
        self.robot_active = 6 # robots active
        self.blf_location = None

    def add_new_frame(self, frame: Frame):
        self.count += 1
        if self.count >= self.update_interval:
            self._version.value += 1
            self.count = 0
        self.frame_list.append(frame)

    def update_geometry(self, geometry: GeometryData):
        self.geometry = geometry
        self.field = geometry.field
        self.ball_model = geometry.models

    def update_gc_data(self,packet):
        t, data = packet[0],packet[1]
        match t:
            case PacketType.ROBOTS_ACTIVE:
                self.update_robots_active(data)
            case PacketType.NEW_STATE:
                self.update_state(data)
            case PacketType.SWITCH_TEAM:
                self.update_team(data["YELLOW"], data["POSITIVE"])
            case PacketType.BLF_LOCATION:
                self.update_ball_left_field_location(data)
            case PacketType.BALL_PLACEMENT_POS:
                self.ball_placement_pos = data

            case _: # if the packet type is unknown
                log.exception(f"undefined Packet - {t}, {data=}")
            
    def update_robots_active(self,new_active) : 
        self.robot_active = new_active
    
    def update_state(self,new_state):
        # when we have a new incoming state, it updates this
        self._state = new_state 

    def update_team(self, us_yellow: bool, us_positive: bool):
        self._us_yellow = us_yellow
        self._us_positive = us_positive

    def update_ball_left_field_location(self, location):
        self.blf_location = location

    def get_ball_placement_pos(self) -> tuple | None:
        return self.ball_placement_pos

    def get_ball_left_field_location(self):
        return self.blf_location
    
    def get_game_state(self):
        return self._state

    def us_yellow(self):
        return self._us_yellow

    def us_positive(self):
        return self._us_positive

    # vision
    def get_latest_frame(self):
        return self.frame_list.latest

    def get_last_n_frames(self, n: int):
        return self.frame_list.get_last_n_frames(n)

    def get_version(self):
        return self._version.value

    # high level
    def get_all_in_team_except(self, us: bool, exclude: list[int]):
        isYellow = self._us_yellow if us is True else not self._us_yellow
        frame = self.frame_list.latest

        if isYellow is True:
            team = frame.robots_yellow
        else:
            team = frame.robots_blue

        if exclude is None or len(exclude) == 0:
            return team
        # now check the list of wanting to be excluded.  
        else: # returning except robot with excluded id
            for e in list(exclude):
                if e in team:
                    team.remove(e)
            return team
        
    # depeciated
    def get_yellow_robots(self,isYellow, robot_id=None) -> object | list:
        raise DeprecationWarning("use frame.get_yellow_robots() instead")
        if isYellow is True:
            if isinstance(robot_id, int):
                return self.frame_list.latest.robots_yellow[robot_id]
            return self.frame_list.latest.robots_yellow
        elif isYellow is False:
            if isinstance(robot_id, int):
                return self.frame_list.latest.robots_blue[robot_id]
            return self.frame_list.latest.robots_blue
        
    # Depeciated
    def get_our_robots(self, us=True, robot_id=None) -> object | list:
        raise DeprecationWarning("use frame.get_yellow_robots() instead")
        frame = self.frame_list.latest
        # set our team or enemy team color
        is_yellow = self._us_yellow if us else not self._us_yellow
        # get the team specified
        robots = frame.robots_yellow if is_yellow else frame.robots_blue
        # return the specific robot or team. 
        return robots[robot_id] if isinstance(robot_id, int) else robots

    def get_active_robots(self):
        return self.robot_active

    # onboard vision — per-robot ball observations from RobotFramework
    def put_onboard_obs(self, obs):
        if obs is None or getattr(obs, "robot_id", -1) < 0:
            return
        self._onboard_store[(bool(obs.is_yellow), int(obs.robot_id))] = obs

    def get_onboard_obs(self, is_yellow, robot_id, max_age=None):
        obs = self._onboard_store.get((bool(is_yellow), int(robot_id)))
        if obs is None:
            return None
        if max_age is not None and (time.time() - obs.recv_ts) > max_age:
            return None
        return obs

    def onboard_snapshot(self):
        return dict(self._onboard_store)