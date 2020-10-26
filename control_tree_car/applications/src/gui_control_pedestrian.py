#!/usr/bin/env python

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry


class MyWindow(Gtk.Window):
    def __init__(self):
        Gtk.Window.__init__(self, title="LGP-Car Control")
        self.set_default_size(200, 200)

        # ros init
        rospy.init_node('gui_control')
        rospy.Subscriber("/lgp_car/odometry", Odometry, self.odometry_callback)
        self.desired_speed = rospy.Publisher('gui_control/lgp_car/desired_speed', Float32, queue_size=10)


	# speed label
        self.speed_label = Gtk.Label()
        self.speed_label.set_text("Desired Speed")

        # speed scale
        ads = Gtk.Adjustment(1, 0, 30, 1, 10, 0)
	self.speed_scale = Gtk.Scale(orientation=Gtk.Orientation.VERTICAL, adjustment=ads, inverted=True)
        self.speed_scale.set_digits(0)
        self.speed_scale.set_vexpand(True)
	self.speed_scale.connect('value-changed', self.on_speed_changed)

        # actual speed label
        self.actual_speed_label = Gtk.Label()
        self.actual_speed_label.set_text("Actual Speed")

	# actual speed
        ads_actual = Gtk.Adjustment(1, 0, 30, 1, 10, 0)
	self.actual_speed_scale = Gtk.Scale(orientation=Gtk.Orientation.VERTICAL, adjustment=ads_actual, inverted=True)
        self.actual_speed_scale.set_digits(0)
        self.actual_speed_scale.set_vexpand(True)
	

        # a grid to attach the widgets
        grid = Gtk.Grid()
        grid.set_column_spacing(10)
        grid.set_column_homogeneous(True)
        grid.attach(self.speed_label, 0, 0, 1, 1)
        grid.attach(self.speed_scale, 0, 1, 1, 1)
        grid.attach(self.actual_speed_label, 1, 0, 1, 1)
        grid.attach(self.actual_speed_scale, 1, 1, 1, 1)
        
        self.add(grid)

    def on_speed_changed(self, widget):
        print("speed:{}".format(self.speed_scale.get_value()))
        self.desired_speed.publish(float(self.speed_scale.get_value()))


    def odometry_callback(self, odometry):
        #print odometry.twist.twist.linear.x
        self.actual_speed_scale.set_value(odometry.twist.twist.linear.x)

win = MyWindow()
win.connect("destroy", Gtk.main_quit)
win.show_all()
Gtk.main()
