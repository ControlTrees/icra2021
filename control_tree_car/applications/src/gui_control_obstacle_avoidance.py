#!/usr/bin/env python

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

import rospy
from std_msgs.msg import Float32

class MyWindow(Gtk.Window):

    def __init__(self):
        Gtk.Window.__init__(self, title="LGP-Car Control")
        self.set_default_size(200, 200)

        # ros init
        rospy.init_node('gui_control')
        self.desired_speed = rospy.Publisher('gui_control/lgp_car/desired_speed', Float32, queue_size=10)
        self.belief_state = rospy.Publisher('gui_control/lgp_car/belief_state', Float32, queue_size=10)

	# speed label
        self.speed_label = Gtk.Label()
        self.speed_label.set_text("Desired Speed")

        # speed scale
        ads = Gtk.Adjustment(10, 0, 15, 1, 10, 0)
	self.speed_scale = Gtk.Scale(orientation=Gtk.Orientation.VERTICAL, adjustment=ads, inverted=True)
        self.speed_scale.set_digits(0)
        self.speed_scale.set_vexpand(True)
	self.speed_scale.connect('value-changed', self.on_speed_changed)

	# bs label
        self.bs_label = Gtk.Label()
        self.bs_label.set_text("Belief state")

        # bs scale
        ad1 = Gtk.Adjustment(0.1, 0, 1.0, 0.01, 10, 0)
	self.bs_scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL, adjustment=ad1, inverted=True)
        #self.bs_scale.set_digits(0)
        self.bs_scale.set_hexpand(True)
        self.bs_scale.connect('value-changed', self.on_bs_changed)

        # a grid to attach the widgets
        grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_column_homogeneous(True)
        grid.attach(self.speed_label, 0, 0, 1, 1)
        grid.attach(self.speed_scale, 0, 1, 1, 1)
        grid.attach(self.bs_label, 0, 2, 1, 1)
        grid.attach(self.bs_scale, 0, 3, 1, 1)
        
        self.add(grid)

    def on_speed_changed(self, widget):
        print("speed:{}".format(self.speed_scale.get_value()))
        self.desired_speed.publish(float(self.speed_scale.get_value()))

    def on_bs_changed(self, widget):
        print("belief state:{}".format(self.bs_scale.get_value()))
        self.belief_state.publish(float(self.bs_scale.get_value()))

win = MyWindow()
win.connect("destroy", Gtk.main_quit)
win.show_all()
Gtk.main()
