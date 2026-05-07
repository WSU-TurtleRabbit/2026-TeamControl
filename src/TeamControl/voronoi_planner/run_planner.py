# from TeamControl.voronoi_planner.planner import VoronoiPlanner
from TeamControl.voronoi_planner.planner_new import VoronoiPlanner
from TeamControl.world.model import WorldModel as wm
from TeamControl.robot.Movement import RobotMovement
from TeamControl.network.robot_command import RobotCommand
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from TeamControl.world.transform_cords import robot2world

# plt.ion()

class PathPlanner():
    # values are from before.
    # CLEARANCE = 100
    # d0 = 1000
    # N = 100
    
    def __init__(self,world_model:wm,dispatcher_q,robot_id):
        self.isYellow = True
        self.robot_id = robot_id
        self.version = 0
        self.wm = world_model
        # field_x, field_y = (5000,2700)
        field_x, field_y = (9000,6000)
        self.timeout = 30
        self.p = VoronoiPlanner(xsize=field_x,ysize=field_y) #initialise planner
        self.output_q = dispatcher_q # output to behaviour tree or world model
        self.goals = [(0, 0)]
        self.movement = RobotMovement()

    def check_wm_update(self):
        # checks if world model has is_yellow otherwise use default here
        self.isYellow = self.wm.us_yellow() if hasattr(self.wm, "us_yellow") else self.isYellow
        # frame version check. 
        new_version = self.wm.get_version() #compares version
        if self.version <= new_version:
            self.version = new_version
            self.frame = self.wm.get_latest_frame() #updates the frame
            return True
        return False
                
    def running (self):
        ## this is for multi processing usage
        robot_id = self.robot_id  # example for robot 0
        start_time = 0
        timeout = self.timeout
        target_pos = 0,0
        new_target = True
        path_planed = False
        # fig, ax = plt.subplots()
        # ax.set_ylim(-2500, 2500)
        # ax.set_xlim(-1400, 1400)
        # plt.show(block=False)
        while True:
            is_updated = self.check_wm_update()
            # follow waypoints here 
            if is_updated is True and self.frame is not None:
                robot = self.frame.get_yellow_robots(isYellow=self.isYellow,robot_id=robot_id)
                if isinstance(robot,int) or self.frame.ball is None:
                    continue
                target_pos1 = self.frame.ball.position # o1r some position
                robot_pos = robot.position

                if not self.close_to_point(a=target_pos,b=target_pos1) : 
                    target_pos = target_pos1
                    self.goals = [target_pos]
                    print("target has changed") 
                    new_target = True
                    path_planed = False
                                       
                # print(f"{target_pos=}")
                self.update_planner(frame=self.frame)
                if new_target or path_planed is False:
                    new_target = False
                    waypoints:list = self.pathplanning()
                    print(f"{waypoints[0]=}, {robot_pos=}, {target_pos=}")
                
                self.p.plot(self.path_obs, [target_pos1], waypoints)

                
                point = waypoints[0][0] if len(waypoints[0])>1 else target_pos
                if len(waypoints[0])>1:
                    path_planed = True
                    start_time = time.time()
                
                if self.close_to_point(a=robot_pos[:2],b=point) and len(waypoints[0]) > 1: 
                    waypoints[0].pop(0) 
                


                vx,vy,w= self.movement.velocity_to_target(robot_pos=robot_pos,target=point,speed=0.1,stop_threshold=70)
                # print(vx,vy)
                command = RobotCommand(robot_id, vx, vy,0,0,0) 
                runtime = 1
                # ax.scatter(target_pos[0], target_pos[1], s=10, c='r', alpha=0.7)
                # transformed = robot2world(robot_pos, p=[vx, vy, 0])
                self.output_q.put((command, runtime))
                # ax.quiver(*robot_pos, transformed.x, transformed.y, scale=21)
                # fig.canvas.draw()
                # fig.canvas.flush_events()
                plt.pause(0.001)
                time.sleep(0.5)
                # output to dispatcher for prototype 
                    # # assuming 0 angular velocity
                # break
                # if isinstance(waypoints, list): # if the waypoint exists
                #     # push forward waypoints to output (back to world model / behaviour tree)
                    #     self.output_q.put((robot_id,waypoints))  

    def close_to_point(self,a:tuple[float,float],b:tuple[float,float],threshold=150):
        '''is point a close to point b'''
        # compares the point array with robot position
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)

        delta = a - b

        return np.dot(delta, delta)  < threshold**2
        
    def update_planner(self,frame):
        
        self.path_obs = [frame.get_yellow_robots(isYellow=self.isYellow,robot_id=self.robot_id).obstacle]
        # obstacles
        our_robot_obs = [r.obstacle for r in frame.get_all_in_team_except(isYellow=self.isYellow, exclude=[])]
        enemy_robot_obs = [r.obstacle for r in frame.get_all_in_team_except(isYellow=not self.isYellow, exclude=[])]
        all_obstacles = our_robot_obs + enemy_robot_obs
        # print("number of Obstacles:",len(all_obstacles))

        self.p.update_obstacles(obstacles=all_obstacles)

    ## this is modified from the example, and I turned it into 1 robot only.
    def pathplanning(self):
        """
        This generates waypoints for all of our robots to target and returns as a list

        Args:
            robot_id (int): id of robot to plan for
            target_pos (tuple[float,float]): targeted location e.g. ball_pos 

        Returns:
            list: list of waypoints (for this robot_id)
        """
        
        path = self.p.do_plan(starting_obs=self.path_obs,ending_points=self.goals)
        # print(f"100 {path=}")
        
        # Print graph

        # print(f"{simplified_paths=}")
        return path # return all waypoints

def run_planner(world_model:wm,dispatcher_q, robot_id):
    planner = PathPlanner(world_model,dispatcher_q,robot_id)
    planner.running()
