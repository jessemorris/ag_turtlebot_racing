#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import ros_numpy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import math
from mesh_visualiser.srv import RequestModelView
from geometry_msgs.msg import Quaternion

class ObjectPlacement():

    def __init__(self):
        self.input_image_topic = rospy.get_param("object_placement/input_camera_topic");

        print(self.input_image_topic)


        rospy.wait_for_service('mesh_visualiser/model_view_bunny')

        self.model_view_service = rospy.ServiceProxy('mesh_visualiser/model_view_bunny',RequestModelView)



        rospy.Subscriber(self.input_image_topic,Image,self.imageCallback)

        self.pub = rospy.Publisher('output_overlayed_image', Image)



        self.cv_image = None
        self.principal_point_x = 640/2
        self.principal_point_y = 480/2
        self.u0 = self.principal_point_x
        self.v0 = self.principal_point_y
        self.fx = 519.566467
        self.fy = 522.066406
        self.distortion_array = np.array([0.187523, -0.355696, 0.002719, -0.003771, 0.000000])

        #k1, k2, k3 is radial distortion
        #k4,k5 is tangential distortion
        self.k4 = self.distortion_array[0]
        self.k5 = self.distortion_array[1]
        self.k1 = self.distortion_array[2]
        self.k2 = self.distortion_array[3]
        self.k3 = self.distortion_array[4]





    def getPixelCoordinates(self):
        #p = point in world frame [x,y,z,0]
        x = 5
        y = 5
        z = 10
        p = np.array([x,y,z,0])

        # R = camera rotation matrix
        # T = camera translation matrix
        R = np.identity(3)

        T = np.array([[0],[0],[0]])



        #K = intrinsic matrix
        K= np.array([[519.566467, 0.000000, 313.837735, 0.000000],
        [0.000000, 522.066406, 248.386084, 0.000000],
        [0.000000, 0.000000, 1.000000, 0.000000]])




        #extrinsic_matrix = [R', T'; 0,0,0,1]
        extrinsic_matrix = np.concatenate((R,T),axis = 1)
        test = np.array([[0,0,0,1]])

        extrinsic_matrix = np.concatenate((extrinsic_matrix,test))


        #
        # np.array[[R,np.transpose(T)],[0,0,0,1]])


        test = np.matmul(K,extrinsic_matrix)

        camera_frame = np.matmul(np.matmul(K,extrinsic_matrix),np.transpose(p))
        # print(camera_frame)


        #these are the undistored points
        u = np.divide(camera_frame[0],camera_frame[2])
        v = np.divide(camera_frame[1],camera_frame[2])


        print("u {}".format(u))
        print("v {}".format(v))

        #u0 = principal_point x
        #v0 = principal_point y

        # fx = focal length x
        # fy = focal length y

        x = (u-self.u0)/self.fx
        y = (v-self.v0)/self.fy

        r = math.sqrt(x**2+y**2)

        #k1, k2, k3 is radial distortion
        #k4,k5 is tangential distortion
        xd = x*(1+self.k1*r**2 +self.k2*r**4 + self.k3*r**6) + 2*self.k4*x*y+self.k5*(r**2+2*x**2);
        yd = y*(1+self.k1*r**2 +self.k2*r**4 + self.k3*r**6)+self.k4*(r**2+2*y**2)+2*self.k5*x*y;

        #distorted pixel coordinates
        ud = self.fx*xd + self.u0;
        vd= self.fy*yd + self.v0;

        print(ud)
        print(vd)




    def imageCallback(self,data):

        # returns image as a numpy array
        self.cv_image = ros_numpy.numpify(data)[... , :3][...,::-1]
        # print(self.cv_image.shape)

        # for each of the objects in the world frame
        self.getPixelCoordinates()

        quaternion = Quaternion(0, 0, 0, 1)
        response = self.model_view_service(quaternion)


        model_image = ros_numpy.numpify(response.image)[... , :4][...,::-1]
        print(model_image.shape)

        h, w, c = self.cv_image.shape


        result = np.zeros((h, w, 3), np.uint8)


        # st = time()
        alpha = model_image[:, :, 3] / 255.0
        result[:, :, 0] = (1. - alpha) * self.cv_image[:, :, 0] + alpha * model_image[:, :, 0]
        result[:, :, 1] = (1. - alpha) * self.cv_image[:, :, 1] + alpha * model_image[:, :, 1]
        result[:, :, 2] = (1. - alpha) * self.cv_image[:, :, 2] + alpha * model_image[:, :, 2]
        # end = time() - st
        # print(end)


        output_image_message = ros_numpy.msgify(Image, result,encoding = '8UC3')
        # cv2.imshow("result", result)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


        self.pub.publish(output_image_message)


if __name__ == '__main__':
    # listener()


    rospy.init_node('object_placement')
    obj = ObjectPlacement()

    rospy.spin()
