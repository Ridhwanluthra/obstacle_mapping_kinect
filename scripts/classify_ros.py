import rospy
from std_msgs.msg import String, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import requests

from resnet50 import ResNet50
from keras.preprocessing import image
from imagenet_utils import preprocess_input, decode_predictions
import numpy as np
import os

# img_path = 'train2014/' + img_path
# img_path = 'ball.jpg'
# img_path = 'face.jpg'
# img_path = 'bag.jpg'
# img_path = 'bottle.jpg'
# img_path = 'room.jpg'

prev = 0
def callback(data):
    if prev == data:
        return
    img = image.load_img(img_path, target_size=(224, 224))
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)

    # print('start')

    preds = model.predict(x)

    print('Predicted:', decode_predictions(preds))
    prev = data


def classify_ros():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('classify_ros', anonymous=True)
    rospy.Subscriber('got_image', )
    # rospy.Subscriber("cluster_distances", Float64MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# runs the listener function if the file is run as a script
if __name__ == '__main__':
    model = ResNet50(weights='imagenet')
    classify_ros()
