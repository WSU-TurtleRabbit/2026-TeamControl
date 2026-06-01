
import time
import math
import matplotlib.pyplot as plt
from TeamControl.SSL.vision.field import FieldLines, Vector2f
from TeamControl.robot.Movement import RobotMovement
from TeamControl.world.transform_cords import world2robot
from TeamControl.robot.striker import buffer_radius
from TeamControl.robot.striker import run_striker 


line_thickness = 1.0              # line thickness
length= 5000.0
width = 3000.0    # half field size
margin = 300.0                     # field margin
penalty_length = 2000.0
penalty_width = 1000.0
center_circle = 500.0
outer_length = length + margin
outer_width = width + margin
arrow_len = 500.0  # mm

def run_test(fn):
    try:
        fn()
        print(f"[PASS] {fn.__name__}")
    except AssertionError as e:
        print(f"[FAIL] {fn.__name__}: {e}")

def test_if_ball_exist():
    robot_pos = (0.0, 0.0, 0.0)
    ball=(100.0, 0.0)

    vx, vy, w = RobotMovement().velocity_to_target(robot_pos=robot_pos, target=ball)

    assert vx != 0.0 or vy != 0.0 or w != 0.0, f"Ball does not exist printed velocities: vx={vx}, vy={vy}, w={w}"
def line(x1, y1, x2, y2):
    
    return FieldLines("", Vector2f(x1, y1), Vector2f(x2, y2), line_thickness)


def plot_ball_to_goal(ax,frame):
    if frame is None:
        return

    # ---- dimensions (mm) ----
    if not hasattr(ax, "field_drawn"):
       

        
        # ---- field lines ----
        field = [
            # main field
            line(-length,  width,  length,  width), # top line
            line(-length, -width,  length, -width), # bottom line
            line(-length, -width, -length,  width), # left line
            line( length, -width,  length,  width), # right line   
            line(0.0, -width, 0.0, width),
            line(-length,  0.0, length, 0.0),

            # left penalty box
            line(-length,  penalty_width, -length + penalty_width,  penalty_width),
            line(-length, -penalty_width, -length + penalty_width, -penalty_width),
            line(-length + penalty_width, -penalty_width, -length + penalty_width,  penalty_width),

            # right penalty box
            line(length,  penalty_width, length - penalty_width,  penalty_width),   # top
            line(length, -penalty_width, length - penalty_width, -penalty_width),   # bottom
            line(length - penalty_width, -penalty_width, length - penalty_width,  penalty_width),

            # left goal
            line(-length-200.0,1000.0/2.0,-length,1000.0/2.0),
            line(-length-200.0,-1000.0/2.0,-length,-1000.0/2.0),
            line(-length-200.0,1000.0/2.0,-length-200.0,-1000.0/2.0),

            line(length,  1000.0/2.0, length + 200.0,  1000.0/2.0),   # top
            line(length, -1000.0/2.0, length + 200, -1000.0/2.0),   # bottom
            line(length + 200.0, 1000.0/2.0, length + 200.0, -1000.0/2.0)  # back vertical
        ]

        # green background (field + margin)
        ax.add_patch(
            plt.Rectangle(
                (-outer_length, -outer_width),
                2.0 * outer_length,
                2.0 * outer_width,
                color="green",
                zorder=0
            )
        )

        for l in field:
            ax.plot([l.p1.x, l.p2.x], [l.p1.y, l.p2.y],
                    linewidth=l.thickness, color="white")

        # center circle
        ax.add_patch(plt.Circle((0, 0), center_circle, fill=False,
                                linewidth=line_thickness, color="white"))

        ax.set_aspect("equal") # keep the field ratio correct
        ax.axis("off")
        ax.field_drawn = True  # mark field as drawn

          
    if frame is not None and frame.ball is not None:
        ball_pos = (frame.ball.x, frame.ball.y)
        us_positive = True
        goal_x = (9000.0 / 2.0) * (1.0 if us_positive else -1.0)
        goal_pos = (goal_x, 0.0)
        robot_pos = (frame.robots_yellow[0].x, frame.robots_yellow[0].y)
        
        robot_orientation = frame.robots_yellow[0].o
        dx = math.cos(robot_orientation) * arrow_len
        dy = math.sin(robot_orientation) * arrow_len

        behind_pos = RobotMovement.behind_ball_point(ball_pos, goal_pos, buffer_radius)
    else:
        return
    
    # ball_pos=(400.0,543.0)
    # goal_pos=(4500.0,0.0)
    # buffer_radius=200.0
    # behind_pos = RobotMovement.behind_ball_point(ball_pos, goal_pos, buffer_radius)


# plotting
    yellow_ball=ax.scatter(*ball_pos, label="Ball", color='yellow')
    ax.scatter(*goal_pos, label="Goal", color='red')
    behind_ball_mark=ax.scatter(*behind_pos, label="Behind ball", marker="x")
    blue_robot=ax.scatter(*robot_pos, label="Robot", color="blue")
    
    blue_arrow=ax.arrow(
    robot_pos[0],
    robot_pos[1],
    dx,
    dy,
    head_width=80,
    length_includes_head=True,
    color="blue")


    red_line,=ax.plot(
        [behind_pos[0], goal_pos[0]],
        [behind_pos[1], goal_pos[1]],
        "r--", )
    ax.grid(True)
    return red_line, yellow_ball, behind_ball_mark, blue_robot, blue_arrow
    

def run_test_to_goal(world_model):
    
    
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 7))
    plot_ball_to_goal(ax,None)
    # ax.legend()
    dynamic_objs = []
    while True:
        frame = world_model.get_latest_frame()
        print(frame)
        if frame is None:
            time.sleep(0.01)
            continue
    # remove old objects
        for obj in dynamic_objs:
            obj.remove()
        # dynamic_objs = []
        
        red_line, yellow_ball, behind_ball_mark, blue_robot, blue_arrow = plot_ball_to_goal(ax, frame)
        dynamic_objs = [red_line, yellow_ball, behind_ball_mark, blue_robot, blue_arrow]
        
        plt.pause(0.1)
        
        # assert frame is not None, "No frame received from vision system."
 
 
    
if __name__ == "__main__":
    from TeamControl.process_workers.wm_runner import WMWorker
    from TeamControl.process_workers.vision_runner import VisionProcess
    from TeamControl.world.model_manager import WorldModelManager,WorldModel
    from TeamControl.utils.Logger import LogSaver
    
    # from multiprocessing.synchronize import Event
    from multiprocessing import Queue, Process,Event
    
    logger = LogSaver()
    vision_q = Queue()
    gc_q = Queue()
    use_grSim = True
    vision_port = 10006
    
    # event : System running ? 
    is_running = Event()
    is_running.set()

    # world model
    wm_manager = WorldModelManager()
    wm_manager.start()
    wm:WorldModel = wm_manager.WorldModel()
    # processes 
    wmr = Process(target=WMWorker.run_worker, args=(is_running,logger,wm,vision_q,gc_q),)
    vision_wkr = Process(target=VisionProcess.run_worker, args=(is_running,logger,vision_q,use_grSim,vision_port),)
    test = Process(target=run_test_to_goal,args=(wm,))
    
    
    vision_wkr.start()
    wmr.start()
    test.start()
    
    while is_running.is_set():
        try:

            print("Type 'exit' to quit: ")
            user_input = input()
            if user_input.lower() == 'exit':
                print("Shutdown signal received...")
                is_running.clear()
                break
        except KeyboardInterrupt:
            print("\nShutdown signal received...")
            is_running.clear()
            # sys.exit()
            
    vision_wkr.join(timeout=5)
    wmr.join(timeout=5)
    test.join(timeout=5)
