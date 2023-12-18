from mapUtilities import *
from a_star import *
from rrt_star import RRTStar
import time
import rclpy
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

POINT_PLANNER=0; A_STAR_PLANNER=1; RRT_PLANNER=2; RRT_STAR_PLANNER=3


# TODO Modify this class so that is uses the RRT* planner with virtual obstacles

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName


    @staticmethod
    def interpolate_path(path: list[list], threshold: float) -> list[list]:
        new_path = [path[0]]

        for i in range(1, len(path)):
            dist = sqrt(((path[i][0] - path[i-1][0])**2) + ((path[i][1] - path[i-1][1])**2))

            if dist > threshold:
                num_interpolated_points = int(dist // threshold)
                interpolated_points = np.linspace(path[i - 1], path[i], num_interpolated_points + 2)[1:-1]
                new_path.extend(interpolated_points)

            new_path.append(path[i])
        
        return new_path

    
    @staticmethod
    def smooth_path(path: np.array, smoothing_factor=0.5, plotting=False) -> np.array:
        # Return original path if too short to spline
        if len(path) <= 3:
            print("Path too short to smooth")
            return path
        # Split into x and y coords
        x_coords, y_coords = path[:, 0], path[:, 1]

        # Perform cubic spline fitting
        tck, _ = splprep([x_coords, y_coords], s=smoothing_factor)
        t_smooth = np.linspace(0, 1, 1000)
        x_smooth, y_smooth = splev(t_smooth, tck)
        smoothed_path = np.column_stack((x_smooth, y_smooth))

        # Plot the original and smoothed paths
        if plotting == True:
            plt.plot(x_coords, y_coords, 'o-', label='Original Path')
            plt.plot(x_smooth, y_smooth, label='Smoothed Path')
            plt.legend()
            plt.xlabel('X-coordinate')
            plt.ylabel('Y-coordinate')
            plt.title('Path Smoothing with Cubic Splines')
            plt.show()

        return smoothed_path


    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner(startPose, endPose)
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self, startPose, endPose):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        # self.m_utilites=mapManipulator(laser_sig=0.4)    
        # self.costMap=self.m_utilites.make_likelihood_field()


        #TODO Remember to initialize the rrt_star
        obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
        
        self.rtt_star = RRTStar(
            max_iter=1500,
            start=startPose,
            goal=endPose,
            obstacle_list=obstacle_list,
            rand_area=[-2, 15],
            expand_dis=1,
            connect_circle_dist=50,
            robot_radius=0.8
        )
        
    
    def trajectory_planner(self, startPoseCart, endPoseCart, type):
        
        #### If using the map, you can leverage on the code below originally implemented for A* (BONUS points option)
        #### If not using the map (no bonus), you can just call the function in rrt_star with the appropriate arguments and get the returned path
        #### then you can put the necessary measure to bypass the map stuff down here.
        # Map scaling factor (to save planning time)
        # scale_factor = 1 # this is the downsample scale, if set 2, it will downsample the map by half, and if set x, it will do the same as 1/x


        # startPose=self.m_utilites.position_2_cell(startPoseCart)
        # endPose=self.m_utilites.position_2_cell(endPoseCart)
        

        start_time = time.time()
        
        # startPose = [int(i/scale_factor) for i in startPose]
        # endPose   = [int(j/scale_factor) for j in endPose]

        # mazeOrigin = self.m_utilites.position_2_cell([0,0])

        # TODO This is for A*, modify this part to use RRT*
        path = self.rtt_star.planning(animation=False)
        if path is None:
            print("Cannot find path")
            return None
        else:
            print("found path!!")

        path.reverse()  # Path is returned backwards otherwise
        path = path[0:-1] # Path contains duplicate goal point
        path = self.interpolate_path(path=path, threshold=2)  # Improves path smoothing performance
        
        end_time = time.time()

        # This will display how much time the search algorithm needed to find a path
        print(f"the time took for rtt_star calculation was {end_time - start_time}")

        # path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]
        # Path = np.array(list(map(self.m_utilites.cell_2_position, path_ )))

        # TODO Smooth the path before returning it to the decision maker
        # this can be in form of a function that you can put in the utilities.py 
        # or add it as a method to the original rrt.py 
        smoothed_path = self.smooth_path(np.array(path), plotting=True)

        return smoothed_path


if __name__=="__main__":
    rclpy.init()

    # m_utilites=mapManipulator()
    
    # map_likelihood=m_utilites.make_likelihood_field()

    # you can do your test here ...

    my_planner = planner(mapName="empty_world", type_=RRT_STAR_PLANNER)
    my_planner.plan([0.319, -0.235], [6, 10])


