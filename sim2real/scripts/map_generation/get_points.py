import sys
import os
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QLabel
from PyQt5.QtGui import QIcon, QPixmap, QImage
from PyQt5 import QtCore
import numpy as np
import argparse
import cv2
from PIL import Image
from numpy.core.fromnumeric import resize
from PIL.ImageQt import ImageQt

def image_resize(image, width_rate=1.0, height_rate=1.0, inter=cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    dim = (int(w * width_rate), int(h * height_rate))

    # resize the image
    resized = cv2.resize(image, dim, interpolation=inter)

    # return the resized image
    return resized

def load_im(im_np):
    im_np = im_np[..., ::-1]  # convert BGR to RGB
    im_np = np.require(im_np, np.uint8, 'C')
    qimage = QImage(im_np.data, im_np.shape[1], im_np.shape[0], im_np.strides[0], QImage.Format_RGB888)
    return qimage

parser = argparse.ArgumentParser()
parser.add_argument(
    "--track_id",
    type=int,
    required=True,
    help="which track image to construct map",
)
parser.add_argument(
    "--width_ratio",
    type = float,
    default = 1.0,
    help = "image width resize rate"
)
parser.add_argument(
    "--height_ratio",
    type = float,
    default = 1.0,
    help = "image height resize rate"
)
parser.add_argument(
    "--map_width",
    type = float,
    default= 20,
    help = "simulator map width"
)
parser.add_argument(
    "--map_height",
    type = float,
    default = 20,
    help = "simulator map height"
)

args = parser.parse_args()

class App(QWidget):

    def __init__(self):
        super().__init__()
        self.title = 'plot track and collect waypoints'
        self.left = 10
        self.top = 10
        self.width = 640
        self.height = 480
        self.first_outer = None
        self.first_inner = None
        self.last_outer = None
        self.last_inner = None
        self.zero = None
        self.mode = 0
        self.inner_loop = 0
        self.outer_loop = 0
        
        self.track_id = args.track_id
        self.width_resize = args.width_ratio
        self.height_resize = args.height_ratio
        self.map_width = args.map_width
        self.map_height = args.map_height
        self.track_name = 'track_{}.png'.format(self.track_id)
        self.initUI(self.track_name)
        self.inner_course = []
        self.outer_course = []
    def print_menu(self):
        print(" ")
        print(" ")
        print(" ")

        print("---------------MENU----------------")

        print("current mode : ", self.mode)
        print("mode 0 : select origin")
        print("mode 1 : draw outer wall")
        print("mode 2 : draw inner wall")
        print("q : see query")
        print("z : cancel last step")
        print("r : reset")
        print("d : select next mode")
        print("a : select previous mode")
        print("space : finish loop")
        print("s : save")
        print("c : don't save and exit")

        print("------------------------------------")

    def remove_point(self, mode):
        print("-------------Removing---------------")
        if self.mode == 0:
           return
        elif self.mode == 1:
           if self.outer_course == []:
              print("Nothing to remove in outer course")
              return
           else:
              tmp = self.outer_course.pop()
              self.last_outer = tmp[:2]
              print(tmp)
              self.outer_loop = 0
        elif self.mode == 2:
           if self.inner_course == []:
              print("Nothing to remove in inner course")
              return
           else:
              tmp = self.inner_course.pop()
              self.last_inner = tmp[:2]
              print(tmp)
              self.inner_loop = 0

    
    def initUI(self, track_name):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
    
        # Create widget
        label = QLabel(self)
        orig_image = cv2.imread(os.path.join('./track_images', track_name))
        qim = load_im(image_resize(orig_image, width_rate = self.width_resize, height_rate = self.height_resize))
        pixmap = QPixmap(qim)
        label.setPixmap(pixmap)
        self.resize(pixmap.width(),pixmap.height())
        self.height = pixmap.height(); self.width = pixmap.width()
        self.show()
        self.print_menu()
    
    def mousePressEvent(self, e):
        print("Input position: {}, {}".format(e.x(), e.y()))
        if e.button() == Qt.RightButton:
            self.remove_point(self.mode)
            print("------------------------------------")
            return
        if self.mode == 0:
            previous_zero = None
            if(self.zero is not None):
               previous_zero = self.zero.copy()
            self.zero = np.array([e.x(), e.y()])

            if previous_zero is not None:
                diff_vector = self.zero - previous_zero
                diff_vector[0] *= self.map_width / self.width
                diff_vector[1] *= self.map_height / self.height
                for point in self.inner_course:
                     point[:2] -= diff_vector
                     point[2:] -= diff_vector
                for point in self.outer_course:
                     point[:2] -= diff_vector
                     point[2:] -= diff_vector

        if self.mode == 1:

            if self.outer_loop == 1:
                 print("Outer loop finished, please undo points to revise")
                 return
            if self.last_outer is None:
                    self.last_outer = np.array([(e.x() - self.zero[0])* self.map_width / self.width, (e.y() - self.zero[1]) * self.map_height / self.height])
                    self.first_outer = self.last_outer
            else:
                self.outer_course.append(np.hstack((self.last_outer, np.array([(e.x() - self.zero[0])* self.map_width / self.width, \
                    (e.y() - self.zero[1]) * self.map_height / self.height]))))
                self.last_outer = np.array([(e.x() - self.zero[0])* self.map_width / self.width, \
                    (e.y() - self.zero[1]) * self.map_height / self.height])
                print(self.outer_course[-1])


        if self.mode == 2:
            if self.inner_loop == 1:
                 print("Inner loop finished, please undo points to revise")
                 return
            if self.last_inner is None:
                    self.last_inner = np.array([(e.x() - self.zero[0])* self.map_width / self.width, (e.y() - self.zero[1]) * self.map_height / self.height])
                    self.first_inner = self.last_inner
            else:
                self.inner_course.append(np.hstack((self.last_inner, np.array([(e.x() - self.zero[0])* self.map_width / self.width, \
                    (e.y() - self.zero[1]) * self.map_height / self.height]))))
                self.last_inner = np.array([(e.x() - self.zero[0])* self.map_width / self.width, \
                    (e.y() - self.zero[1]) * self.map_height / self.height])
                print(self.inner_course[-1])


        self.print_menu()


    
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Space:  # Minus
           if self.mode == 1:
            self.outer_course.append(np.hstack((self.last_outer, self.first_outer)))
            print(self.outer_course[-1])

            self.last_outer = self.first_outer.copy()
            self.outer_loop = 1
            print("Outer loop done")
           if self.mode == 2:
            self.inner_course.append(np.hstack((self.last_inner, self.first_inner)))
            print(self.inner_course[-1])
            self.last_inner = self.first_inner.copy()
            self.inner_loop = 1
            print("Inner loop done")

         

        if event.key() == QtCore.Qt.Key.Key_A:
            self.mode = max(self.mode - 1, 0)



        if event.key() == QtCore.Qt.Key.Key_D:
            self.mode = min(self.mode + 1, 2)

        if event.key() == QtCore.Qt.Key.Key_Q:
            if self.zero is None:
               print("------------------------")
               print("Nothing Yet...")
               print("------------------------")
               self.print_menu()
               return
            print("------------------------")
            print('zero : ', self.zero)
            print("------------------------")
            print('outer course : ')
            for point in self.outer_course:
               print(point)
            print("------------------------")
            print('inner course : ')
            for point in self.inner_course:
               print(point)
            print("------------------------")


        if event.key() == QtCore.Qt.Key.Key_R:
            self.inner_course = []
            self.outer_course = []
            self.first_outer = None
            self.first_inner = None
            self.last_outer = None
            self.last_inner = None
            self.zero = None
            self.mode = 0
            self.inner_loop = 0
            self.outer_loop = 0
        if event.key() == QtCore.Qt.Key.Key_S:
            if self.outer_loop != 1:
                print("Please finish outer loop")
            elif self.inner_loop != 1:
                print("Please finish inner loop")
            with open("../../worlds/world_map/track_{}.txt".format(self.track_id), 'wb') as f:
                np.savetxt(f, np.vstack((np.array(self.outer_course), np.array(self.inner_course))), fmt = "%.3e")
                print("SAVED!!!!")
            #self.close()
        if event.key() == QtCore.Qt.Key.Key_C:
            self.close()
        self.print_menu()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
