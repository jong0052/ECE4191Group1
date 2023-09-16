from multiprocessing import Manager

class MPManager:
    def __init__(self):
        manager = Manager()

        self.poses = manager.list() # For Plotting
        self.obstacle_data = manager.list() # Map.obstacle_logs, each point is [x, y] for each obstacle dot
        self.plan_mp = manager.list() # rrt_plan, each point is [x, y, th] for waypoints.
        self.robot_data = manager.list() # Robot position, [x, y, theta]
        self.goal_data = manager.list() # Current GoalSetter data, [x, y, theta]
        self.us_data = manager.list() # Ultrasonic Distance Data. (US_ID, point)

        # Multiprocessing
        self.wl_goal_value = 0 # Left wheel goal (set by navigation loop). RPM.
        self.wr_goal_value = 0 # Right wheel goal (set by navigation loop). RPM.
        self.current_wl = 0 # Current Left wheel speed (set by serial loop). RPM. 
        self.current_wr = 0 # Current Right wheel speed (set by serial loop). RPM.

        # This needs improvement.
        # Suggest implementation: List where each point contains (US_ID, point). 
        # When US_LOOP reads a value, add a point to this list.
        # After that, when navigation loop reads it, it adds the point and removes it from the list.
        # This means only the most updated value is accepted (since old values are discarded), therefore
        # we don't need to deal with update jank.
        # US_ID can even be replaced with x, y, th relative positions from center for better readings.
        self.usLeft_value = 100
        self.usLeft_update = 0
        self.usFront_value = 100
        self.usFront_update = 0
        self.usRight_value = 100
        self.usRight_update = 0
