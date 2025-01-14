#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray, UInt32
import numpy as np
import colorsys
import time
import matplotlib.pyplot as plt


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.colour_callback)
        self.line_sub = rospy.Subscriber("line_idx", UInt32, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes
        self.colour_map = colour_map
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour
        self.pastcolours = []

        # New variables
        self.line_idx = 0
        self.map = ["y", "g", "b", "o", "o", "g", "b", "o", "y", "g", "b"] # shifted down by 2
        self.num_updates = 0 # track number of state updates done
        self.measure_state = [-1] # previous states


    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = colorsys.rgb_to_hsv(*(np.array(msg.data)/255.0))  # [r, g, b]


    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.line_idx = msg.data


    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()


    def state_model(self, u):
        """
        State model was implemented within the state_predict function.
        """
        pass


    def measurement_model(self, x):
        """
        Measurement model was implemented within the the state_update function.
        """
        pass


    def state_predict(self, uk, xk, zk):
        rospy.loginfo("predicting state")

        tb_state_model = [[0.85, 0.1, 0.05],
                       [0.05, 0.9, 0.05],
                       [0.05, 0.1, 0.85]] # index uk first then xk+1

        preds = []
        for i in range(len(self.map)):
            p_xkp1_zk = 0
            for j in range(len(self.map)):
                if abs(j - i) <= 1:
                    p_state = tb_state_model[uk][i-j+1]
                elif j == 0 and i == 10:
                    p_state = tb_state_model[uk][0]
                elif i == 0 and j == 10:
                    p_state = tb_state_model[uk][2]
                else:
                    p_state = 0
                
                p_xkp1_zk += p_state * self.probability[j]
            preds.append(p_xkp1_zk)

        return preds


    def state_update(self, uk, xk, zk, preds):
        rospy.loginfo("updating state")

        tb_msmt_model = [[0.7, 0.2, 0.00, 0.00, 0.1],
                      [0.2, 0.7, 0.0, 0.0, 0.1],
                      [0.0, 0.1, 0.7, 0.1, 0.1],
                      [0.0, 0.0, 0.2, 0.7, 0.1]] # index xk first then zk
        
        col_2_idx = {"o": 0, "g": 1, "b": 2, "y": 3, "n": 4}
        idx_2_col = {0: "o", 1: "g", 2: "b", 3: "y", 4: "n"}

        update = []
        if zk == None:
            update = preds
        else:
            norm = 0
            for i in range(len(preds)):
                norm += tb_msmt_model[col_2_idx[self.map[i]]][zk] * preds[i]

            #norm += 0.0001
            for i in range(len(preds)):
                p_xkp1_zkp1 = (tb_msmt_model[col_2_idx[self.map[i]]][zk] * preds[i])/norm
                update.append(p_xkp1_zkp1)

        return update


    def match_colour(self, arr_col, codes):
        errs = []
        error_weighting = [1.3, 1, 2, 1, 1.25, 1.25] #orange, green, blue, yellow, line, floor
        for i in range(len(codes)):
            errs.append(error_weighting[i] * np.linalg.norm(np.array(codes[i]) - np.array(arr_col)))
        
        index_min = min(range(len(errs)), key=errs.__getitem__)

        
        if (index_min == 4 or index_min == 5) and errs[index_min] >= 40:
            errs[index_min] = 9999
            return min(range(len(errs)), key=errs.__getitem__)

        return index_min
    

    def bay_loc(self, zk):
        # Variables
        uk = 2
        xk = max(range(len(self.probability)), 
            key=self.probability.__getitem__) 
            # estimate state with largest probability
        #zk passed in
        
        # State prediction
        pred_prob = self.state_predict(uk, xk, zk)

        # State update
        update_prob = self.state_update(uk, xk, zk, pred_prob)

        # Store new state and update probability
        self.probability = update_prob
    
    
    def state_measure(self, count, colourcodes):
        """ GO FISH (checking for continuous colours) """
        self.pastcolours.append(self.cur_colour)
        if len(self.pastcolours) > 100:
            self.pastcolours = self.pastcolours[-50:-1]

        if np.count_nonzero(np.array(self.pastcolours) == -1) > len(self.pastcolours) - count:
            return

        match_col = self.match_colour(self.pastcolours[-1], colourcodes)
        for i in range(2, count):
            if match_col != self.match_colour(self.pastcolours[-i], colourcodes):
                return
        col = match_col
        
        # Append if new state
        if  col != self.measure_state[-1]:
            self.measure_state.append(col)

            # Check trigger for state update
            if col != 4 and col != 5:
                self.num_updates += 1
                print("UPDATE UPDATE UPDATE")
                self.bay_loc(col)


    def deliver(self):
        cmd = Twist()
        stack = cmd.linear.x
        cmd.linear.x = 0
        rate = rospy.Rate(10)

        cmd.angular.z = math.pi/12
        self.cmd_pub.publish(cmd)

        time_loop = time.time()
        print("Turning Turning Turning")
        while True:
            if (time.time() - time_loop) * math.pi/12 >= math.pi/2:
                break

            self.cmd_pub.publish(cmd)
            rate.sleep()


        time_loop = time.time()
        print("Delivering Delivering Delivering")
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)
        while True:
            if (time.time() - time_loop) >= 1:
                break
            self.cmd_pub.publish(cmd)
            rate.sleep()
        

        cmd.angular.z = -math.pi/12
        time_loop = time.time()
        print("Turning Turning Turning")
        while True:
            if (time.time() - time_loop) * math.pi/12 >= math.pi/2:
                break

            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        cmd.linear.x = stack
        cmd.angular.z = 0

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 1, 2, 0, 0, 1, 2, 0, 3, 1, 2]

    # Colour values
    colour_codes_sunny_rgb = [
        [233, 118, 104],  # orange
        [160, 185, 156],  # green
        [178, 164, 184],  # blue
        [181, 164, 145],  # yellow
        [143, 131, 135],  # line
        #[180, 166, 167],  # floor
    ]

    colour_codes_cloudy_rgb = [
        [222, 100, 81],  # orange
        [153, 170, 155],  # green
        [170, 156, 177],  # blue
        [179, 157, 144],  # yellow
        [143, 128, 132],  # line
        #[180, 166, 167],  # floor
    ]

    colour_codes_night_rgb = [
        [252, 146, 91],  # orange
        [147, 161, 160],  # green
        [170, 159, 180],  # blue
        [181, 161, 147],  # yellow
        [143, 128, 134],  # line
        #[180, 166, 167],  # floor
    ]

    colour_codes_night_hsv = [
        [0.024, 0.636, 0.868],  # orange
        [0.39, 0.08, 0.708],  # green
        [0.78, 0.16, 0.69],  # blue
        [0.065, 0.19, 0.74],  # yellow
        [0.94, 0.1, 0.572],  # line
        [0.05, 0.1, 0.572]  # line
        #[180, 166, 167],  # floor
    ]

    colour_codes = [
        [0.020, 0.636, 0.87],  # orange
        [0.32, 0.08, 0.708],  # green
        [0.78, 0.16, 0.69],  # blue
        [0.09, 0.20, 0.75],  # yellow
        [0.94, 0.09, 0.617],  # line
        [0.05, 0.1, 0.60]  # line
        #[180, 166, 167],  # floor
    ]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    # New variables
    idx2col = {-1: "none", 0: "orange", 1: "green", 2: "blue", 3: "yellow", 4: "line", 5: "line", 6: "floor"}

    cmd = Twist()
    desired = 320
    correction, error, integral, lasterror = 0, 0, [], 0
    kp, ki, kd = 0.003, 0.0006, 0.005

    goal = [0, 1, 7]
    print("Current goal:", goal[0])

    while not rospy.is_shutdown():

        localizer.wait_for_colour()

        ##################### State Update ####################
        localizer.state_measure(10, colour_codes)

        # Print state robot thinks it is at and probability
        print(max(range(len(localizer.probability)), key=localizer.probability.__getitem__)+2, 
              localizer.probability[max(range(len(localizer.probability)), key=localizer.probability.__getitem__)])
        
        # Check if goal office reached
        if localizer.probability[goal[0]] >= 0.60 and localizer.num_updates >= 5:
            localizer.deliver()

            goal.pop(0)

            if len(goal) == 0:
                print("DONE DONE DONE")
                exit()

        #################### PID Control ####################
        
        # Print colour robot thinks it is reading and what the raw colour measurements are
        print(idx2col[localizer.match_colour(localizer.cur_colour, colour_codes)], localizer.cur_colour)

        # Control
        if localizer.match_colour(localizer.cur_colour, colour_codes) == 4 or \
            localizer.match_colour(localizer.cur_colour, colour_codes) == 5:
            # Correction calculation
            error = desired - localizer.line_idx
            integral.append(error)
            sum_integral = sum(integral)
            if (sum_integral <= 0.0001):
                sum_integral = 0.0001

            correction = error*kp + (sum_integral/abs(sum_integral))*min(abs(sum_integral), 500)*ki + (error-lasterror)*kd

            lasterror = error

            integral = integral[-300:]

            # Publish correction command
            cmd.linear.x = 0.06
            cmd.angular.z = correction
    
        else:
            cmd.linear.x = 0.06
            cmd.angular.z = 0.0
           
        localizer.cmd_pub.publish(cmd)
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
