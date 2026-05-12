import numpy as np

class PIDController:
    """PID controller for throttle (velocity tracking)."""

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        
        derivative=(error-self.prev_error)/dt
        self.integral+=error*dt
        self.prev_error=error
        output=self.Kd*derivative+self.Kp*error+self.Ki*self.integral
        return output





def pure_pursuit_steering(x,y,yaw,v,waypoints,L,k_dd,ld_min,max_steer):
    # Step 1: Compute the lookahead distance based on speed
    #   ld = k_dd * v + ld_min
    ld=k_dd*v+ld_min

    # Step 2: Find the nearest waypoint to the rear axle
    #   Compute distances from (x, y) to every waypoint, pick the closest index.
    pts=np.array([[p[0],p[1]] for p in waypoints])
    car=np.array([x,y])
    dist=np.linalg.norm(pts-car,axis=1)
    closest=np.argmin(dist)

    # Step 3: Search forward from the nearest waypoint to find the goal point
    #   Starting from the nearest index, walk forward along the waypoints until
    #   you find the first waypoint whose distance from (x, y) >= ld.
    #   If none is found, use the last waypoint in the array.
    target=waypoints[-1]
    for i in range(closest,len(waypoints)):
        dx=waypoints[i][0] - x
        dy=waypoints[i][1] - y
        if np.hypot(dx,dy) > ld:
            target=waypoints[i]
            break

    # Step 4: Compute alpha — the angle between the vehicle heading and the
    #   direction from the rear axle to the goal point.
    #   alpha = arctan2(goal_y - y, goal_x - x) - yaw
    dx = target[0] - x
    dy = target[1] - y
    alpha = np.arctan2(dy, dx) - yaw
    alpha = np.arctan2(np.sin(alpha),np.cos(alpha))

    # Step 5: Compute the steering angle using the Pure Pursuit formula:
    #   steer = arctan(2 * L * sin(alpha) / ld)
    steer = np.arctan2(2 * L * np.sin(alpha), ld)# write your code here

    # Step 6: Clip steering to max_steer
    steer = np.clip(steer, -max_steer,max_steer)# clip to max_steer

    return steer, target
   
    