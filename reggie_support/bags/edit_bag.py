import rosbag
import copy

with rosbag.Bag('/home/stevie/reggie_ws/src/reggie/reggie_support/bags/new_realsense.bag', 'w') as outbag:
    for topic, msg, t in rosbag.Bag('/home/stevie/reggie_ws/src/reggie/reggie_support/bags/realsense.bag').read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf_static":
            transforms = copy.deepcopy(msg.transforms)
            for i in transforms:
                print(msg.transforms)
                print("======")
                if i.child_frame_id not in ["camera_depth_optical_frame"]: 
                    #print(i)
                    #print("------")
                    msg.transforms.remove(i)
            print(len(msg.transforms))
            if len(msg.transforms) > 0:
                print('goin for it')
                outbag.write(topic, msg, t)
        else:
            outbag.write(topic, msg, t)
