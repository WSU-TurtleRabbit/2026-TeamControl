
from enum import Enum, IntEnum,auto

import numpy as np

# these are Enums defined by us

class PacketType(Enum): # for message sending onto gc_queue
    ROBOTS_ACTIVE = auto()
    NEW_STATE = auto()
    SWITCH_TEAM = auto()
    BLF_LOCATION = auto()
    BALL_PLACEMENT_POS = auto()  # designated_position from referee message
    GC_STATUS = auto()           # full referee status snapshot

class GameState(Enum):
    HALTED = auto()
    STOPPED = auto()
    RUNNING = auto()
    PREPARE_KICKOFF = auto()
    OPP_KICKOFF = auto()

    FREE_KICK = auto()
    OPP_FREE_KICK = auto()
    BALL_PLACEMENT = auto()
    KICKOFF = auto()
    
    HALF_TIME = auto()
    # TIME_OUT = auto()
    
    PREPARE_PENALTY = auto()
    PREPARE_PENALTY_OPP = auto()
    PENALTY_SHOOT = auto()
    PENALTY_DEFEND = auto()
    

"""
The following is obtained from gc_referee_message.proto
"""
class Command(Enum):
    HALT = 0
    STOP = 1
    NORMAL_START = 2
    FORCE_START = 3
    
    PREPARE_KICKOFF_YELLOW = 4
    PREPARE_KICKOFF_BLUE = 5
    PREPARE_PENALTY_YELLOW = 6
    PREPARE_PENALTY_BLUE = 7
    DIRECT_FREE_YELLOW = 8
    DIRECT_FREE_BLUE = 9
    INDIRECT_FREE_YELLOW = 10
    INDIRECT_FREE_BLUE = 11
    
    TIMEOUT_YELLOW = 12
    TIMEOUT_BLUE = 13
    GOAL_YELLOW = 14
    GOAL_BLUE = 15
    BALL_PLACEMENT_YELLOW = 16
    BALL_PLACEMENT_BLUE = 17

class Team(Enum):
    UNKNOWN = 0
    YELLOW = 1
    BLUE = 2
    
class GameEventType(Enum):
    UNKNOWN_GAME_EVENT_TYPE = 0

    BALL_LEFT_FIELD_TOUCH_LINE = 6
    BALL_LEFT_FIELD_GOAL_LINE = 7
    AIMLESS_KICK = 11

    ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19
    DEFENDER_IN_DEFENSE_AREA = 31
    BOUNDARY_CROSSING = 41
    KEEPER_HELD_BALL = 13
    BOT_DRIBBLED_BALL_TOO_FAR = 17

    BOT_PUSHED_BOT = 24
    BOT_HELD_BALL_DELIBERATELY = 26
    BOT_TIPPED_OVER = 27
    BOT_DROPPED_PARTS = 47

    ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15
    BOT_KICKED_BALL_TOO_FAST = 18
    BOT_CRASH_UNIQUE = 22
    BOT_CRASH_DRAWN = 21

    DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29
    BOT_TOO_FAST_IN_STOP = 28
    BOT_INTERFERED_PLACEMENT = 20
    EXCESSIVE_BOT_SUBSTITUTION = 48

    POSSIBLE_GOAL = 39
    GOAL = 8
    INVALID_GOAL = 42

    ATTACKER_DOUBLE_TOUCHED_BALL = 14
    PLACEMENT_SUCCEEDED = 5
    PENALTY_KICK_FAILED = 43

    NO_PROGRESS_IN_GAME = 2
    PLACEMENT_FAILED = 3
    MULTIPLE_CARDS = 32
    MULTIPLE_FOULS = 34
    BOT_SUBSTITUTION = 37
    TOO_MANY_ROBOTS = 38
    CHALLENGE_FLAG = 44
    CHALLENGE_FLAG_HANDLED = 46
    EMERGENCY_STOP = 45

    UNSPORTING_BEHAVIOR_MAJOR = 36
    UNSPORTING_BEHAVIOR_MINOR = 35

class Stage(Enum):
    NORMAL_FIRST_HALF_PRE = 0
    NORMAL_FIRST_HALF = 1
    NORMAL_HALF_TIME = 2
    NORMAL_SECOND_HALF_PRE = 3
    NORMAL_SECOND_HALF = 4
    EXTRA_TIME_BREAK = 5
    EXTRA_FIRST_HALF_PRE = 6
    EXTRA_FIRST_HALF = 7
    EXTRA_HALF_TIME = 8
    EXTRA_SECOND_HALF_PRE = 9
    EXTRA_SECOND_HALF = 10
    PENALTY_SHOOTOUT_BREAK = 11
    PENALTY_SHOOTOUT = 12
    POST_GAME = 13

class MatchType(Enum):
    UNKNOWN_MATCH = 0
    GROUP_PHASE = 1
    ELIMINATION_PHASE = 2
    FRIENDLY = 3
    
class Point():
    def __init__(self,point):
        # required
       self.x = float(point.x)
       self.y = float(point.y)
       self.vector = np.array([self.x, self.y])
       

def has_proto_field(obj, field_name:str):
    try:
        return obj.HasField(field_name)
    except ValueError:
        return False
    

    # def match_event(self,game_event):
    #     ## access the corresponding event attribute
    #     event = getattr(game_event, self.event, None)
    #     if event is None:
    #         print(f"Warning: Event {self.event} not found in game_event")
    #         return
    #     match self.type:
    #         # STOPPING - Ball out of Field events
    #         case GameEventType.BALL_LEFT_FIELD_TOUCH_LINE:
    #             return self.ball_left_field(event)
    #         case GameEventType.BALL_LEFT_FIELD_GOAL_LINE:
    #             return self.ball_left_field(event)
    #         case GameEventType.AIMLESS_KICK:
    #             return self.aimless_kick(event)
            
    #         #Stopping Fouls
    #         case GameEventType.ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA:
    #             return self.attacker_touched_ball_in_defense_area(event)
    #         case GameEventType.DEFENDER_IN_DEFENSE_AREA:
    #             return self.defender_in_defense_area_partially(event)
    #         case GameEventType.BOUNDARY_CROSSING:
    #             return self.boundary_crossing(event)
    #         case GameEventType.KEEPER_HELD_BALL:
    #             return self.keeper_held_ball(event)
    #         case GameEventType.BOT_DRIBBLED_BALL_TOO_FAR:
    #             return self.bot_dribbled_ball_too_far(event)

    #         case GameEventType.BOT_PUSHED_BOT:
    #             return self.bot_push_bot(event)
    #         case GameEventType.BOT_HELD_BALL_DELIBERATELY:
    #             return self.bot_held_ball_deliberately(event)
    #         case GameEventType.BOT_TIPPED_OVER:
    #             return self.bot_tipped_over(event)
    #         case GameEventType.BOT_DROPPED_PARTS:
    #             return self.bot_dropped_parts(event)
            
    #         # non stopping fouls
    #         case GameEventType.ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA:
    #             return self.attacker_too_close_to_defense_area(event)
    #         case GameEventType.BOT_KICKED_BALL_TOO_FAST:
    #             return self.bot_kicked_ball_too_fast(event)
    #         case GameEventType.BOT_CRASH_UNIQUE:
    #             return self.bot_crash_unique(event)
    #         case GameEventType.BOT_CRASH_DRAWN:
    #             return self.bot_crash_drawn(event)
            
    #         # fouls while ball out of play
    #         case GameEventType.DEFENDER_TOO_CLOSE_TO_KICK_POINT:
    #             return self.defender_too_close_to_kick_point(event)
    #         case GameEventType.BOT_TOO_FAST_IN_STOP:
    #             return self.bot_too_fast_in_stop(event)
    #         case GameEventType.BOT_INTERFERED_PLACEMENT:
    #             return self.bot_interfered_placement(event)
            
    #         # Scoring Goals
    #         case GameEventType.POSSIBLE_GOAL:
    #             return self.goal(event)
    #         case GameEventType.GOAL:
    #             return self.goal(event)
    #         case GameEventType.INVALID_GOAL:
    #             return self.goal(event)
            
    #         # Other Events
    #         case GameEventType.ATTACKER_DOUBLE_TOUCHED_BALL:
    #             return self.attacker_double_touched_ball(event)
    #         case GameEventType.PLACEMENT_SUCCEEDED:
    #             return self.placement_succeeded
    #         case GameEventType.PENALTY_KICK_FAILED:
    #             return self.penalty_kick_failed(event)
            
    #         case GameEventType.NO_PROGRESS_IN_GAME:
    #             return self.no_progress_in_game(event)
    #         case GameEventType.PLACEMENT_FAILED:
    #             return self.placement_failed(event)
    #         case GameEventType.MULTIPLE_CARDS:
    #             return self.multiple_cards(event)
    #         case GameEventType.MULTIPLE_FOULS:
    #             return self.multiple_fouls(event)
    #         case GameEventType.BOT_SUBSTITUTION:
    #             return self.bot_substitution(event)
    #         case GameEventType.EXCESSIVE_BOT_SUBSTITUTION:
    #             return self.excessive_bot_substitution(event)
    #         case GameEventType.TOO_MANY_ROBOTS:
    #             return self.too_many_robots(event)
    #         case GameEventType.CHALLENGE_FLAG:
    #             return self.challenge_flag(event)
    #         case GameEventType.CHALLENGE_FLAG_HANDLED:
    #             return self.challenge_flag_handled(event)
    #         case GameEventType.EMERGENCY_STOP:
    #             return self.emergency_stop(event)
            
    #         case GameEventType.UNSPORTING_BEHAVIOR_MINOR:
    #             return self.unsporting_behavior_minor(event)
    #         case GameEventType.UNSPORTING_BEHAVIOR_MAJOR:
    #             return self.unsporting_behavior_major(event)
