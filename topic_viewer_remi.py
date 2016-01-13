#!/usr/bin/env python

import remi.gui as gui
from remi import App, start
import rospy
from rostopic import get_topic_class

DEFAULT_WIDTH = 800
DEFAULT_HEIGHT = 25


def get_html_repr_of_msg(msg, deepness=0, python_pasteable_output=False):
    html_str = ''
    if hasattr(msg, '__slots__'):
        # print "msg has attr __slots__: " + str(msg.__slots__)
        for slot_name, slot_type in zip(msg.__slots__, msg._slot_types):
            # print "  Doing slot: " + str(slot_name) + " of type: " + str(slot_type)
            # print "    Writting: " + deepness * '&nbsp;&nbsp;' +  '<b>' + slot_name + '</b>= '
            if python_pasteable_output:
                html_str += deepness * '&nbsp;&nbsp;' + '<b>' + slot_name + '</b>= '
            else:
                html_str += deepness * '&nbsp;&nbsp;' + '<b>' + slot_name + '</b>: '
            subfield = msg.__getattribute__(slot_name)
            if hasattr(msg.__getattribute__(slot_name), '__slots__'):
                # print "  Which has subslots!"
                # print "    Writting: " + '<br>&nbsp;&nbsp;'
                html_str += '<br>'
                html_str += get_html_repr_of_msg(subfield, deepness+1)
            else:
                if slot_type.endswith('[]') and slot_type.find('/') > -1:
                    html_str += '<br>'
                    for array_elem in subfield:
                        html_str += deepness * '&nbsp;&nbsp;' + '-<br>'
                        html_str += get_html_repr_of_msg(array_elem, deepness+1)
                else:
                # print "  No subslots, directly type: " + str(type(subfield))
                # print "    Writting: " + str(subfield) + '<br>'
                    if type(subfield) == str:
                        html_str += "'" + subfield + "'<br>"
                    else:
                        html_str += str(subfield) + '<br>'
    # else:
    #     if type(msg) == str:
    #         html_str += "'" + msg + "'<br>"
    #     else:
    #         html_str += str(msg) + '<br>'
    return html_str


class MyApp(App):

    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def main(self, name='world'):
        self.topic_sub = None
        # the arguments are	width - height - layoutOrientationHorizontal
        self.wid = gui.Widget(-1, -1, gui.Widget.LAYOUT_VERTICAL, 20)

        self.hor_topics = gui.Widget(-1, -1, gui.Widget.LAYOUT_HORIZONTAL, 20)
        self.bt = gui.Button(-1, -1, 'Refresh topics list')
        self.subscribed_to_topic = False
        self.current_topic = ''
        self.bt_pause = gui.Button(-1, -1, 'Pause topic')
        self.bt_pause.set_on_click_listener(self, 'pause_subscriber')
        self.bt.set_on_click_listener(self, 'refresh_topics')

        self.hor_topics.append(1, self.bt)
        self.hor_topics.append(3, self.bt_pause)

        # This makes the button not be left
        self.bt.style['display'] = 'block'
        self.bt.style['margin'] = '10px auto'
        self.bt.style['float'] = 'none'

        self.bt_pause.style['display'] = 'block'
        self.bt_pause.style['margin'] = '10px auto'
        self.bt_pause.style['float'] = 'none'

        self.topic_content_label = gui.Label(-1, -1, "Topic data.")
        # self.topic_content_label = gui.TextInput(DEFAULT_WIDTH, -1, single_line=False)
        # self.topic_content_label.set_text("Topic data.")
        #self.topic_content_label.style['text-align'] = 'center'

        self.refresh_topics()
        self.hor_topics.style['padding-bottom'] = '20px'

        self.wid.append(2, self.topic_content_label)

        # returning the root widget
        return self.wid

    def pause_subscriber(self):
        if self.subscribed_to_topic:
            self.unsubscribe_topic()
            self.bt_pause.set_text("Resume topic")
        else:
            self.subscribe_to_topic(self.current_topic)
            self.bt_pause.set_text("Pause topic")

    def subscribe_to_topic(self, topic_name):
        topic_type, real_topic_name, _ = get_topic_class(topic_name)
        self.topic_sub = rospy.Subscriber(topic_name, topic_type,
                                          self.topic_cb, queue_size=1)
        self.subscribed_to_topic = True
        self.current_topic = topic_name
        rospy.loginfo("Subscribed to " + self.current_topic +
                      " and type: " + topic_type)
        self.bt_pause.set_text("Pause topic")

    def unsubscribe_topic(self):
        self.topic_sub.unregister()
        self.subscribed_to_topic = False




    def topic_cb(self, msg):
        # rospy.loginfo("\n\n###################### cb: " + str(msg) + "###############################################################\n\n")
        # msg_as_str = str(msg)
        # msg_html = msg_as_str.replace('\n', '<br>')
        # msg_html = msg_html.replace(' ', '&nbsp;')
        html_msg = get_html_repr_of_msg(msg)
        self.topic_content_label.set_text(html_msg)

    def on_dropdown_change(self, value):
        print "Dropdown changed to: " + str(value)
        # If we had a previous client, disconnect it
        if self.topic_sub is not None:
            self.unsubscribe_topic()
        if value == "Choose topic...":
            self.topic_content_label.set_text("Topic data.")
            return

        # Get new client
        self.subscribe_to_topic(value)

        # This must be done later on! HACK! (append sets a margin)
        # This makes the table not stick left, but float in the middle
        self.topic_content_label.style['margin'] = '10px auto'

    def refresh_topics(self):
        self.published_topics_and_types = rospy.get_published_topics()
        self.published_topics = [topic_name for topic_name, topic_type in self.published_topics_and_types]
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
        self.hor_topics.append(2, self.dropdown)
        # This makes the dropdown not be left
        self.dropdown.style['display'] = 'block'
        self.dropdown.style['margin'] = '10px auto'
        self.dropdown.style['float'] = 'none'
        self.wid.append(1, self.hor_topics)

if __name__ == "__main__":
    rospy.init_node('web_topic_viewer')

    start(MyApp,
          address="0.0.0.0",
          port=8091,
          multiple_instance=True,
          update_interval=0.1,
          start_browser=False,
          debug=False)
