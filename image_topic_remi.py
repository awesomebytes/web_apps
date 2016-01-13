#!/usr/bin/env python
"""
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
"""

import time
import io
import remi.gui as gui
from remi import start, App
import rospy
from sensor_msgs.msg import Image, CompressedImage
from rostopic import get_topic_type
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError



class CompressedImageVideoWidget(gui.Image):
    def __init__(self, width, height, fps=5):
        #super(CompressedImageVideoWidget, self).__init__(width, height, "/"+str(id(self))+"/get_image_data")
        #super(CompressedImageVideoWidget, self).__init__(width, height, "/%s/get_image_data")
        super(CompressedImageVideoWidget, self).__init__(width, height, "/res/logo.png")
        self.fps = fps
        self.last_img = None
        self.last_frame = None
        self.img_w = 400
        self.img_h = 400
        self.subscriber = None
        self.last_topic = None

        javascript_code = gui.Tag()
        javascript_code.type = 'script'
        javascript_code.attributes['type'] = 'text/javascript'
        javascript_code.append( 'code' , """
            function update_image%(id)s(){
                console.debug('on update_image');
                if(document.getElementById('%(id)s').getAttribute('play')=='False'){
                    console.debug('not showing anything');
                    return;
                    }

                var url = '/%(id)s/get_image_data';
                var xhr = new XMLHttpRequest();
                xhr.open('GET', url, true);
                xhr.responseType = 'blob';
                xhr.onload = function(e){
                    console.debug('on xhr.onload');
                    var urlCreator = window.URL || window.webkitURL;
                    var imageUrl = urlCreator.createObjectURL(this.response);
                    document.getElementById('%(id)s').src = imageUrl;
                }
                xhr.onerror = function(e){
                    console.debug('on xhr.onerror ');
                }
                xhr.send();
            };

            setInterval( update_image%(id)s, %(update_rate)s );
            """ % {'id': id(self), 'update_rate': 1000/self.fps})

        self.append('javascript_code', javascript_code)
        self.stop()

    def image_cb(self, msg):
        self.new_img = True
        self.last_img = msg

    def play(self):
        if self.subscriber is None:
            self.subscribe(self.last_topic)
        self.attributes['play'] = True
        self.style['width'] = str(self.img_w)+'px'
        self.style['height'] = str(self.img_h)+'px'
        self.style['float'] = 'none'
        self.style['margin'] = '10px auto'
        self.style['display'] = 'block'

    def stop(self):
        self.attributes['play'] = False
        # self.style['width'] = '400px'
        # self.style['height'] = '400px'
        self.unsubscribe()

    def get_image_data(self):
        if self.last_img is None:
            rospy.logwarn("We have no image yet...")
            return None, None
            # while not rospy.is_shutdown() and self.last_img is None:
            #     rospy.sleep(0.1)

        if self.subscriber.type is None:
            return None, None

        if self.subscriber.type == 'sensor_msgs/CompressedImage':
            frame = self.last_img.data
        elif self.subscriber.type == 'sensor_msgs/Image':
            if not self.new_img:
                frame = self.last_frame
            cvImage = self.bridge.imgmsg_to_cv2(self.last_img, "bgr8")
            frame = np.array(cv2.imencode('.jpg', cvImage)[1]).tostring()
            self.last_frame = frame
        headers = {'Content-type': 'image/jpeg'}
        self.new_img = False
        return [frame, headers]

    def subscribe(self, topic):
        if topic is None:
            rospy.logerr("Got topic None, not subscribing")
            return
        if self.subscriber is not None:
            self.subscriber.unregister()
        self.last_img = None
        type_name, topic_name, _ = get_topic_type(topic)
        if type_name == 'sensor_msgs/Image':
            self.subscriber = rospy.Subscriber(topic, Image, self.image_cb, queue_size=1)
            self.bridge = CvBridge()
        elif type_name == 'sensor_msgs/CompressedImage':
            self.subscriber = rospy.Subscriber(topic, CompressedImage, self.image_cb, queue_size=1)
        else:
            rospy.logerr('Topic is not of image type')
            return
        while not rospy.is_shutdown() and self.last_img is None:
            rospy.sleep(0.1)
        if type_name == 'sensor_msgs/Image':
            w = self.last_img.width
            h = self.last_img.height
        elif type_name == 'sensor_msgs/CompressedImage':
            w, h = self.get_compressed_image_size(self.last_img)
        self.img_w = w
        self.img_h = h
        self.last_topic = topic
        # self.play()

    def unsubscribe(self):
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
            self.last_img = None

    def get_compressed_image_size(self, compressed_image):
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        height = image_np.shape[0]
        width = image_np.shape[1]
        return width, height

class MyApp(App):

    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def main(self, name='world'):
        # the arguments are	width - height - layoutOrientationOrizontal
        self.wid = gui.Widget(800, 800, False, 10)

        self.hor_topics = gui.Widget(-1, -1, gui.Widget.LAYOUT_HORIZONTAL, 20)
        # Refresh topics
        self.bt = gui.Button(-1, -1, 'Refresh topics list')
        self.bt.set_on_click_listener(self, 'refresh_topics')

        # List non-compressed topics checkbox
        self.check_raw_topics = False
        self.checkbox = gui.CheckBoxLabel(200, 30, label='Show raw topics', checked=False)
        self.checkbox.set_on_change_listener(self, 'change_raw_topics')

        # Stop/Play button
        self.playing = False
        self.bt_pause = gui.Button(-1, -1, 'Stop video')
        self.bt_pause.set_on_click_listener(self, 'pause_play')

        self.hor_topics.append(1, self.bt)

        self.hor_topics.append(2, self.checkbox)

        # This makes the button not be left
        self.bt.style['display'] = 'block'
        self.bt.style['margin'] = '10px auto'
        self.bt.style['float'] = 'none'

        self.rosvideo_widget = None
        
        # Dropdown of topics
        self.refresh_topics()

        # Pause button
        self.hor_topics.append(4, self.bt_pause)

        self.bt_pause.style['display'] = 'block'
        self.bt_pause.style['margin'] = '10px auto'
        self.bt_pause.style['float'] = 'none'

        # Dropdown of topic FPS #TODO:
        self.fps = 20

        # Put the Widget already or it gives problem to put it dynamically
        self.rosvideo_widget = CompressedImageVideoWidget(400, 400, fps=self.fps)
        self.wid.append(2, self.rosvideo_widget)      

        rospy.loginfo("Finished initialization.")

        # returning the root widget
        return self.wid

    def change_raw_topics(self, value):
        print "checkbox raw topics: " + str(value) + " type: " + str(type(value))
        if value == 'true':
            self.check_raw_topics = True
        elif value == 'false':
            self.check_raw_topics = False
        else:
            rospy.logerr("Checkbox value was not 'true' or 'false'")

    def refresh_topics(self):
        self.published_topics_and_types = rospy.get_published_topics()
        if not self.check_raw_topics:
            self.published_topics = [topic_name for topic_name, topic_type in self.published_topics_and_types if topic_type == 'sensor_msgs/CompressedImage']
        else:
            self.published_topics = [topic_name for topic_name, topic_type in self.published_topics_and_types if (topic_type == 'sensor_msgs/CompressedImage' or topic_type == 'sensor_msgs/Image')]
        self.published_topics.sort()
        rospy.loginfo("Found topics:\n" +
                      str(self.published_topics))
        self.dropdown = gui.DropDown(-1, -1)
        choose_ddi = gui.DropDownItem(-1, -1, "Choose topic...")
        self.dropdown.append(0, choose_ddi)
        for idx, topic_name in enumerate(self.published_topics):
            ddi = gui.DropDownItem(-1, -1, topic_name)
            self.dropdown.append(idx+1, ddi)

        self.dropdown.set_on_change_listener(self, 'on_dropdown_change')
        # using ID 2 to update the dropdown
        self.hor_topics.append(3, self.dropdown)
        # This makes the dropdown not be left
        self.dropdown.style['display'] = 'block'
        self.dropdown.style['margin'] = '10px auto'
        self.dropdown.style['float'] = 'none'
        # Force to re-render the pause button after the topics list
        self.hor_topics.append(4, self.bt_pause)
        self.bt_pause.style['display'] = 'block'
        self.bt_pause.style['margin'] = '10px auto'
        self.bt_pause.style['float'] = 'none'

        self.wid.append(1, self.hor_topics)
        # Re-render
        if self.rosvideo_widget:
            self.wid.append(2, self.rosvideo_widget)


    def on_dropdown_change(self, value):
        print "Dropdown changed to: " + value
        if value == "Choose topic...":
            if self.playing:
                self.rosvideo_widget.stop()
                self.playing = False
            return
        self.rosvideo_widget.subscribe(value)
        self.rosvideo_widget.play()
        self.playing = True

    def on_fps_dropdown_change(self, value):
        pass

    def pause_play(self):
        if self.playing:
            self.bt_pause.set_text("Play video")
            self.rosvideo_widget.stop()
            self.playing = False
        else:
            self.bt_pause.set_text("Stop video")
            self.rosvideo_widget.play()
            self.playing = True



if __name__ == "__main__":
    # optional parameters
    # start(MyApp,address='127.0.0.1', port=8081, multiple_instance=False,enable_file_cache=True, update_interval=0.1, start_browser=True)
    rospy.init_node('view_topic_remi')
    start(MyApp, address='0.0.0.0', port=8093, debug=True)
