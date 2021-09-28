#!/usr/bin/env python
# license removed for brevity
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped

def publish_path():

    pub = rospy.Publisher('/uav1/decompose_node/path_in', Path, queue_size=10)
    rospy.init_node('path_publisher', anonymous=True)
     
    key = raw_input("press a key to start estimation")
   
    init_path = Path()
    init_path.header.frame_id = "uav1/global_origin"
    pa = Point()
    pb = Point()
    psa = PoseStamped()
    psb = PoseStamped()
    psv = []
   
    waypoints = []
    # waypoints.append((-10.0, 0.0, 2.0)) 
    # # waypoints.append((-5.0, 0.0, 1.0)) 
    # waypoints.append((4.0, 0.0, 1.0)) 
    # waypoints.append((3.0, 0.0, 10.0)) 
    # waypoints.append((3.0, 0.0, 20.0)) 
    # # waypoints.append((5.0, 5.0, 20.0)) 
    # waypoints.append((-5.0, 5.0, 20.0)) 
    # waypoints.append((-5.0, -5.0, 20.0)) 
    # waypoints.append((-10.0, 0.0, 4.0)) 
    waypoints.append((-9.0, 0.0, 4.0)) 
    waypoints.append((-8.0, 0.0, 4.0)) 
    waypoints.append((-7.0, 0.0, 4.0)) 
    waypoints.append((-6.0, 0.0, 4.0)) 
    waypoints.append((-5.0, 0.0, 4.0)) 
    waypoints.append((-4.0, 0.0, 4.0)) 
    waypoints.append((-3.0, 0.0, 4.0)) 
    waypoints.append((-2.0, 0.0, 4.0)) 
    waypoints.append((-1.0, 0.0, 4.0)) 
    waypoints.append((-0.0, 0.0, 4.0)) 
    waypoints.append((1.0, 0.0, 4.0)) 
    waypoints.append((2.0, 0.0, 4.0)) 
    waypoints.append((3.0, 0.0, 4.0)) 
    waypoints.append((4.0, 0.0, 4.0)) 
    waypoints.append((5.0, 0.0, 4.0)) 
    waypoints.append((6.0, 0.0, 4.0)) 
    waypoints.append((7.0, 0.0, 4.0)) 
    waypoints.append((8.0, 0.0, 4.0)) 
    waypoints.append((9.0, 0.0, 4.0)) 
    waypoints.append((10.0, 0.0, 4.0)) 
    # waypoints.append((8.099894e+00, -1.741087e+01, 1.989040e+00))                
    # waypoints.append((8.111768e+00, -1.731334e+01, 1.986940e+00))                
    # waypoints.append((8.122577e+00, -1.721992e+01, 1.984766e+00))                
    # waypoints.append((8.132320e+00, -1.713078e+01, 1.982604e+00))                
    # waypoints.append((8.140895e+00, -1.704613e+01, 1.980511e+00))                
    # waypoints.append((8.148140e+00, -1.696619e+01, 1.978505e+00))                
    # waypoints.append((8.153864e+00, -1.689121e+01, 1.976570e+00))                
    # waypoints.append((8.160170e+00, -1.681931e+01, 1.974526e+00))                
    # waypoints.append((8.168854e+00, -1.674866e+01, 1.972305e+00))                
    # waypoints.append((8.179122e+00, -1.667949e+01, 1.970065e+00))                
    # waypoints.append((8.190220e+00, -1.661201e+01, 1.968015e+00))                
    # waypoints.append((8.201462e+00, -1.654632e+01, 1.966396e+00))                
    # waypoints.append((8.212174e+00, -1.648251e+01, 1.965297e+00))                
    # waypoints.append((8.221669e+00, -1.642072e+01, 1.964770e+00))                
    # waypoints.append((8.229236e+00, -1.636094e+01, 1.964828e+00))                
    # waypoints.append((8.234207e+00, -1.630274e+01, 1.965352e+00))                
    # waypoints.append((8.236069e+00, -1.624587e+01, 1.966105e+00))                
    # waypoints.append((8.234527e+00, -1.619049e+01, 1.966833e+00))                
    # waypoints.append((8.229568e+00, -1.613679e+01, 1.967389e+00))                
    # waypoints.append((8.221193e+00, -1.608505e+01, 1.967621e+00))                
    # waypoints.append((8.209303e+00, -1.603556e+01, 1.967330e+00))                
    # waypoints.append((8.193826e+00, -1.598862e+01, 1.966405e+00))                
    # waypoints.append((8.174685e+00, -1.594454e+01, 1.964890e+00))                
    # waypoints.append((8.151789e+00, -1.590366e+01, 1.962984e+00))                
    # waypoints.append((8.125018e+00, -1.586631e+01, 1.961001e+00))                
    # waypoints.append((8.094223e+00, -1.583282e+01, 1.959304e+00))                
    # waypoints.append((8.059219e+00, -1.580345e+01, 1.958207e+00))                
    # waypoints.append((8.019782e+00, -1.577837e+01, 1.957733e+00))                
    # waypoints.append((7.975738e+00, -1.575763e+01, 1.957604e+00))                
    # waypoints.append((7.926939e+00, -1.574111e+01, 1.957582e+00))                
    # waypoints.append((7.873061e+00, -1.572862e+01, 1.957714e+00))                
    # waypoints.append((7.813856e+00, -1.572012e+01, 1.957853e+00))                
    # waypoints.append((7.749347e+00, -1.571564e+01, 1.957638e+00))                
    # waypoints.append((7.679565e+00, -1.571494e+01, 1.957000e+00))                
    # waypoints.append((7.604462e+00, -1.571763e+01, 1.956100e+00))                
    # waypoints.append((7.523996e+00, -1.572330e+01, 1.955130e+00))                
    # waypoints.append((7.438227e+00, -1.573158e+01, 1.954207e+00))                
    # waypoints.append((7.347336e+00, -1.574213e+01, 1.953402e+00))                
    # waypoints.append((7.251657e+00, -1.575464e+01, 1.952780e+00))                
    # waypoints.append((7.151694e+00, -1.576882e+01, 1.952514e+00))                

    for w in waypoints:
        p = Point()
        p.x = w[0]
        p.y = w[1]
        p.z = w[2]
        ps = PoseStamped()
        ps.pose.position = p
        psv.append(ps)

    init_path.poses = psv
    pub.publish(init_path)

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
