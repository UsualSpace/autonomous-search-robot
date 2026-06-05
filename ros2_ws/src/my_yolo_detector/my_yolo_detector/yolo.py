from ultralytics import YOLO
import rclpy 
from rclpy.node import Node
#used to subscribe to camera for image
from sensor_msgs.msg import Image
#Allows for opencv images from ros
from cv_bridge import CvBridge

#getting the raw value data type from vision msg
from vision_msgs.msg import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Bool
'''
This class represents the yolo node that will be used for ros2.
Purpose: Initailize the node, and connect the connection between this node and camera.
Inheritance: This node inherits from the ROS2 Node class, which servers as the single point of entry for essiantal communication features.
output: annotated image and raw detection data

'''
class YoloNode(Node):
    #initialize the node
    def __init__(self):
        super().__init__("YOLO_NODE")
        #variables
        #yolo model name
        self.model_name = 'yolo12s.pt'

        #topic name that can be seen in topic list
        #in
        self.topic_in = '/robot/camera/image'
        #out
        self.topic_out = '/detections/image_annotated'
        self.topic_out_detection = '/detections/raw'
        self.topic_out_found = "/detections/found"
        #to control robot movement
        self.topic_cmd_vel = "/cmd_vel"

        #Declaring defult varaible name for pass in parameters
        self.declare_parameter('target_param','default')

        #gettin in target object from user
        self.target_object = str(self.get_parameter('target_param').value).lower().strip()
        #state flag for loops
        self.found_flag = False

        #outputs input on command line
        if self.target_object is not None:
            self.get_logger().info(f'Recieved user  input {self.target_object}')
        else:
            self.get_logger().info(f'Did not recieved any arguements')

        #loading in the model and model weight
        self.get_logger().info(f"Yolo model is being loaded: {self.model_name}.")
        try:
            self.model = YOLO(self.model_name)
            self.class_names = self.model.names
            self.get_logger().info("Model was successfully loaded.")
        except Exception as e:
            self.get_logger().error(f"failed to load in model: {e}.")
            rclpy.shutdown()
            return

        # intialize cv bridge. This helps turn ros2 output into cv images j
        self.bridge = CvBridge()
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # only keep the latest frame, drop stale ones
        ) 
        #creating the subscriber to the path of the camera
        self.subscription = self.create_subscription(
            Image,
            self.topic_in,
            #nonblocking allows to run in parrellel with other program
            self.image_callback,
            qos
        )

        self.get_logger().info(f"Node intialized and subrcribe to: {self.topic_in}")

        #creating the published
        self.image_pub = self.create_publisher(Image,self.topic_out,10)

        #creating the detection publish which is a list of 2d arrays
        self.detection_pub = self.create_publisher(Detection2DArray,self.topic_out_detection,10)
        
        #publish that the robot will throught if image is found
        self.detection_signal = self.create_publisher(Twist,self.topic_cmd_vel,10)

        self.found_signal_pub = self.create_publisher(Bool,self.topic_out_found,10)


    def image_callback(self,msg):
        '''
        Purpose: that will recieve data and will take care of handling
        Input: self, and msg which is data recieved which is type Image as shown in subscriber specified above
        Ouput: Display a image in a window
        '''
        #will not show by default.. Extra steps required for this to work
        self.get_logger().debug("Recieved Image")

        try:
            #convert ros image to opencv image
            cv_image = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except Exception as e:
            self.get_logger().error(f"failed to convert image {e}")
            return

        #preform yolo detection 
        results = self.model(cv_image, verbose=False)
        result = results[0] #get the first result

        #visualize the result
        pub_frame = result.plot()

        #this is for the publisher that will help nodes talk with each other.
        #converts cv image back to ros image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(pub_frame,"bgr8")
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"failed to convert image: {e}")
            return

        #publish the raw detection
        detections_array = Detection2DArray()
        #Creating a new message, the header is the original header from the camera. Essentially the frame from camera
        detections_array.header = msg.header

        #state flag
        current_frame = False

        #logic that will iterater through each object detected throught the current trip
        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy() #(N,4)
            confs = result.boxes.conf.cpu().numpy() #(N,)
            classes = result.boxes.cls.cpu().numpy() #(N,)

            for i in range(len(boxes)):
                cls_idx = int(classes[i])
                detected_name = self.class_names[cls_idx].lower().strip()
                score = float(confs[i])

                #checks if string matches terminal arguement and satisfies a relieable confidence rating of 55% minimum
                if detected_name == self.target_object and score >= 0.40:
                    current_frame = True
                if detected_name != self.target_object:
                    continue

                xmin,ymin,xmax,ymax = boxes[i]

                #create Detection2D
                detection = Detection2D()
                detection.header = msg.header #syncing the timeframe with original image data

                #Getting the bounding box
                bbox = BoundingBox2D()

                #using Point2d for center
                
                bbox.center.position.x = float((xmin + xmax) / 2.0) #getting the width
                bbox.center.position.y = float((ymin + ymax) / 2.0) #getting the height
                bbox.size_x = float(xmax - xmin)
                bbox.size_y = float(ymax - ymin)

                detection.bbox = bbox                

               # getting the object class and score
                obj_hyp = ObjectHypothesisWithPose()
                
                # Double guard classification index mapping
                cls_idx = int(classes[i])
                if cls_idx in self.class_names:
                    obj_hyp.hypothesis.class_id = str(self.class_names[cls_idx])
                else:
                    obj_hyp.hypothesis.class_id = f"unknown_{cls_idx}"
                    
                obj_hyp.hypothesis.score = score
                detection.results.append(obj_hyp)
                #else add to array of object found
                detections_array.detections.append(detection)

        #publishing the complete array
        self.detection_pub.publish(detections_array)
        #checking to see if object found
        if current_frame and not self.found_flag:
            self.found_flag = True
            self.execute_shutdown()
    
    
    def execute_shutdown(self):
        '''
        Function that will halt robots movement and breaks out main thread execution
        '''

        #stop robotA
        bool_msg = Bool()
        bool_msg.data = True
        self.found_signal_pub.publish(bool_msg)
        stop_msg = Twist()  # all zeros by default
        self.detection_signal.publish(stop_msg)


def main(args=None):
    #initailizing ros2 communication
    rclpy.init(args=args)

    #creating a ros2 Node
    node = YoloNode()

    try:

        #node will run,until killed
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info("Shutting down")
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down")
    finally:
        #cleaning up
        node.destroy_node()
        #shutdown ros2 communication
        rclpy.shutdown()



if __name__ == "__main__":
    main()
