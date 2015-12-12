#!/usr/bin/env python

#
# Copyright (c) 2014 Miguel Sarabia
# Imperial College London
#
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#

#===============================================================================
# IMPORTS
#===============================================================================
import os
import rosbag
import argparse
import glob
import cv2
import cv_bridge


#===============================================================================
# BAG EXTRACTOR
#===============================================================================
class BagExtractor:
    def __init__(self, filename, prefix):
        self.video_path = (
            prefix + 
            os.path.splitext(os.path.basename( filename ) )[0] +
            ".mpeg" )
        self.bag = rosbag.Bag(filename)
        self.bridge = cv_bridge.CvBridge()
        self.stills = []
        self.fps = 30


    def extract_images(self):
        for topic, msg, t in self.bag.read_messages(topics=["/camera"] ):
            self.stills.append(
                ( self.bridge.imgmsg_to_cv2(msg, "bgr8"), t.to_sec()) )

    def save_video(self):
        video = cv2.VideoWriter(
            self.video_path, 
            cv2.cv.CV_FOURCC('M','P','E','G'),
            self.fps,
            (160,120) )

        period = 1.0 / self.fps
        video_time = 0.0
        start_exp_time = self.stills[0][1]
        next_frame_time = self.stills[1][1] - start_exp_time
        image = self.stills[0][0]
        frame = 0
        while True:
            if video_time >= next_frame_time:
                frame += 1
                
                if frame +1 >= len(self.stills):
                    break
                
                next_frame_time = self.stills[frame+1][1] - start_exp_time
                image = self.stills[frame][0]
             
            video.write(image)
            video_time += period
        
        cv2.destroyAllWindows()
        video.release()


    def close(self):
        self.valid = False
        self.bag.close()


#===============================================================================
# MAIN
#===============================================================================
if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description = "Extract videos from rosbags")
    argparser.add_argument("-p", "--prefix", help = "Prefix of output files", default = "")
    argparser.add_argument("--dir", help = "Change current directory to DIR")
    args = argparser.parse_args()

    if args.dir:
        os.chdir(args.dir)

    extracted_data = []

    # Get all bags
    for filename in glob.glob("*nao_puppet*.bag"):
        print("Loading file: " + filename)
        bag = BagExtractor(filename, args.prefix)
        
        print("Extracting images from: " + filename)
        bag.extract_images()
        
        print("Saving video to: " + bag.video_path)
        bag.save_video()
        
        print("")
        # Close bag
        bag.close()

