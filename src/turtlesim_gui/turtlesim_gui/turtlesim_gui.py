import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, simpledialog
from turtlesim.srv import Spawn, Kill, SetPen, TeleportAbsolute, TeleportRelative
# Import parameter service types
from rcl_interfaces.srv import DescribeParameters, GetParameterTypes, GetParameters, ListParameters, SetParameters, SetParametersAtomically
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue as ParameterValueMsg # Added imports
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
# Import Action related interfaces
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute
from action_msgs.msg import GoalStatus

import re
import threading
import sys
import math

class TurtlesimGUI(Node):
    def __init__(self):
        super().__init__('turtlesim_gui')
        self.window = tk.Tk()
        self.window.title("Turtlesim GUI")
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

        # --- ROS2 Clients and State ---
        self.selected_turtle = None
        self.twist_publishers = {}
        self.pose_subscribers = {}
        self.current_pose = {}
        self.parameter_clients = {} # Clients for parameter services
        self.action_clients = {} # Clients for action services
        self.current_action_goal = None # To keep track of the active action goal
        self.parameter_list = [] # To store the list of parameters

        # --- GUI Widgets ---
        self.create_widgets()

        # --- Timers ---
        self.update_timer = self.create_timer(2.0, self.update_lists) # Timer to update lists every 2 seconds
        self.parameter_update_timer = self.create_timer(5.0, self.list_parameters_periodic) # Timer to update parameter list


    def create_widgets(self):
        # Use frames to organize the layout
        control_frame = ttk.LabelFrame(self.window, text="Turtlesim Control")
        control_frame.grid(row=0, column=0, padx=5, pady=5, sticky=tk.N+tk.W+tk.E)

        movement_frame = ttk.LabelFrame(self.window, text="Turtle Movement")
        movement_frame.grid(row=1, column=0, padx=5, pady=5, sticky=tk.N+tk.W+tk.E)

        pen_frame = ttk.LabelFrame(self.window, text="Pen Control (Selected Turtle)")
        pen_frame.grid(row=2, column=0, padx=5, pady=5, sticky=tk.N+tk.W+tk.E)

        teleport_frame = ttk.LabelFrame(self.window, text="Teleport (Selected Turtle)")
        teleport_frame.grid(row=3, column=0, padx=5, pady=5, sticky=tk.N+tk.W+tk.E)

        parameter_frame = ttk.LabelFrame(self.window, text="Parameter Control")
        parameter_frame.grid(row=0, column=1, rowspan=4, padx=5, pady=5, sticky=tk.N+tk.W+tk.E+tk.S)

        action_frame = ttk.LabelFrame(self.window, text="Action Control (Selected Turtle)")
        action_frame.grid(row=4, column=0, padx=5, pady=5, sticky=tk.N+tk.W+tk.E)

        info_frame = ttk.LabelFrame(self.window, text="ROS Information")
        info_frame.grid(row=0, column=2, rowspan=5, padx=5, pady=5, sticky=tk.N+tk.W+tk.E+tk.S)

        details_frame = ttk.LabelFrame(self.window, text="Details and Output")
        details_frame.grid(row=5, column=0, columnspan=3, padx=5, pady=5, sticky=tk.N+tk.W+tk.E+tk.S)


        # --- Control Frame Widgets ---
        # Row 0: Buttons
        self.spawn_button = tk.Button(control_frame, text="Spawn Turtle", command=self.spawn_turtle)
        self.spawn_button.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        self.kill_button = tk.Button(control_frame, text="Kill Selected Turtle", command=self.kill_turtle)
        self.kill_button.grid(row=0, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        self.reset_button = tk.Button(control_frame, text="Reset Turtlesim", command=self.reset_turtlesim)
        self.reset_button.grid(row=0, column=2, padx=2, pady=2, sticky=tk.W+tk.E)

        # Row 1: Custom spawn position inputs
        self.spawn_custom_label = tk.Label(control_frame, text="Spawn at position:")
        self.spawn_custom_label.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W)

        # Row 2: X, Y, Theta inputs
        self.spawn_x_label = tk.Label(control_frame, text="X:")
        self.spawn_x_label.grid(row=2, column=0, padx=2, pady=2, sticky=tk.W)
        self.spawn_x_entry = tk.Entry(control_frame, width=5)
        self.spawn_x_entry.grid(row=2, column=0, padx=25, pady=2, sticky=tk.W)
        self.spawn_x_entry.insert(0, "5.0")  # Default value

        self.spawn_y_label = tk.Label(control_frame, text="Y:")
        self.spawn_y_label.grid(row=2, column=1, padx=2, pady=2, sticky=tk.W)
        self.spawn_y_entry = tk.Entry(control_frame, width=5)
        self.spawn_y_entry.grid(row=2, column=1, padx=25, pady=2, sticky=tk.W)
        self.spawn_y_entry.insert(0, "5.0")  # Default value

        self.spawn_theta_label = tk.Label(control_frame, text="Theta:")
        self.spawn_theta_label.grid(row=2, column=2, padx=2, pady=2, sticky=tk.W)
        self.spawn_theta_entry = tk.Entry(control_frame, width=5)
        self.spawn_theta_entry.grid(row=2, column=2, padx=40, pady=2, sticky=tk.W)
        self.spawn_theta_entry.insert(0, "0.0")  # Default value

        # Row 3: Custom spawn button
        self.spawn_custom_button = tk.Button(control_frame, text="Spawn at Position", command=self.spawn_turtle_at_position)
        self.spawn_custom_button.grid(row=3, column=0, columnspan=3, padx=2, pady=2, sticky=tk.W+tk.E)

        control_frame.grid_columnconfigure(0, weight=1)
        control_frame.grid_columnconfigure(1, weight=1)
        control_frame.grid_columnconfigure(2, weight=1)


        # --- Movement Frame Widgets ---
        self.forward_button = tk.Button(movement_frame, text="Forward", command=self.move_forward)
        self.forward_button.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        self.backward_button = tk.Button(movement_frame, text="Backward", command=self.move_backward)
        self.backward_button.grid(row=0, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        self.left_button = tk.Button(movement_frame, text="Left", command=self.turn_left)
        self.left_button.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        self.right_button = tk.Button(movement_frame, text="Right", command=self.turn_right)
        self.right_button.grid(row=1, column=1, padx=2, pady=2, sticky=tk.W+tk.E)

        movement_frame.grid_columnconfigure(0, weight=1)
        movement_frame.grid_columnconfigure(1, weight=1)


        # --- Pen Frame Widgets ---
        self.pen_r_label = tk.Label(pen_frame, text="R:")
        self.pen_r_label.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.pen_r_entry = tk.Entry(pen_frame, width=5)
        self.pen_r_entry.grid(row=0, column=1, padx=2, pady=2, sticky=tk.W)

        self.pen_g_label = tk.Label(pen_frame, text="G:")
        self.pen_g_label.grid(row=0, column=2, padx=2, pady=2, sticky=tk.W)
        self.pen_g_entry = tk.Entry(pen_frame, width=5)
        self.pen_g_entry.grid(row=0, column=3, padx=2, pady=2, sticky=tk.W)

        self.pen_b_label = tk.Label(pen_frame, text="B:")
        self.pen_b_label.grid(row=0, column=4, padx=2, pady=2, sticky=tk.W)
        self.pen_b_entry = tk.Entry(pen_frame, width=5)
        self.pen_b_entry.grid(row=0, column=5, padx=2, pady=2, sticky=tk.W)

        self.pen_width_label = tk.Label(pen_frame, text="Width:")
        self.pen_width_label.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W)
        self.pen_width_entry = tk.Entry(pen_frame, width=5)
        self.pen_width_entry.grid(row=1, column=1, padx=2, pady=2, sticky=tk.W)

        self.pen_off_var = tk.IntVar()
        self.pen_off_check = tk.Checkbutton(pen_frame, text="Pen Off", variable=self.pen_off_var)
        self.pen_off_check.grid(row=1, column=2, columnspan=2, padx=2, pady=2, sticky=tk.W)

        self.set_pen_button = tk.Button(pen_frame, text="Set Pen", command=self.set_pen)
        self.set_pen_button.grid(row=1, column=4, columnspan=2, padx=2, pady=2, sticky=tk.W+tk.E)

        for i in range(6):
             pen_frame.grid_columnconfigure(i, weight=1)


        # --- Teleport Frame Widgets ---
        self.teleport_abs_label = tk.Label(teleport_frame, text="Absolute:")
        self.teleport_abs_label.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_x_label = tk.Label(teleport_frame, text="X:")
        self.teleport_abs_x_label.grid(row=0, column=1, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_x_entry = tk.Entry(teleport_frame, width=5)
        self.teleport_abs_x_entry.grid(row=0, column=2, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_y_label = tk.Label(teleport_frame, text="Y:")
        self.teleport_abs_y_label.grid(row=0, column=3, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_y_entry = tk.Entry(teleport_frame, width=5)
        self.teleport_abs_y_entry.grid(row=0, column=4, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_theta_label = tk.Label(teleport_frame, text="Theta (rad):")
        self.teleport_abs_theta_label.grid(row=0, column=5, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_theta_entry = tk.Entry(teleport_frame, width=5)
        self.teleport_abs_theta_entry.grid(row=0, column=6, padx=2, pady=2, sticky=tk.W)
        self.teleport_abs_button = tk.Button(teleport_frame, text="Teleport Absolute", command=self.teleport_absolute)
        self.teleport_abs_button.grid(row=0, column=7, padx=2, pady=2, sticky=tk.W+tk.E)

        self.teleport_rel_label = tk.Label(teleport_frame, text="Relative:")
        self.teleport_rel_label.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W)
        self.teleport_rel_linear_label = tk.Label(teleport_frame, text="Linear:")
        self.teleport_rel_linear_label.grid(row=1, column=1, padx=2, pady=2, sticky=tk.W)
        self.teleport_rel_linear_entry = tk.Entry(teleport_frame, width=5)
        self.teleport_rel_linear_entry.grid(row=1, column=2, padx=2, pady=2, sticky=tk.W)
        self.teleport_rel_angular_label = tk.Label(teleport_frame, text="Angular (rad):")
        self.teleport_rel_angular_label.grid(row=1, column=3, padx=2, pady=2, sticky=tk.W)
        self.teleport_rel_angular_entry = tk.Entry(teleport_frame, width=5)
        self.teleport_rel_angular_entry.grid(row=1, column=4, padx=2, pady=2, sticky=tk.W)
        self.teleport_rel_button = tk.Button(teleport_frame, text="Teleport Relative", command=self.teleport_relative)
        self.teleport_rel_button.grid(row=1, column=7, padx=2, pady=2, sticky=tk.W+tk.E)

        for i in range(8):
             teleport_frame.grid_columnconfigure(i, weight=1)


        # --- Parameter Frame Widgets ---
        self.parameter_listbox_label = tk.Label(parameter_frame, text="Parameters:")
        self.parameter_listbox_label.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.parameter_listbox = tk.Listbox(parameter_frame, height=8)
        self.parameter_listbox.grid(row=1, column=0, columnspan=3, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.parameter_listbox.bind('<<ListboxSelect>>', self.show_parameter_details)

        self.list_parameters_button = tk.Button(parameter_frame, text="List Parameters", command=self.list_parameters)
        self.list_parameters_button.grid(row=2, column=0, padx=2, pady=2, sticky=tk.W+tk.E)
        self.get_parameters_button = tk.Button(parameter_frame, text="Get Selected Parameter(s)", command=self.get_selected_parameters)
        self.get_parameters_button.grid(row=2, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        self.describe_parameters_button = tk.Button(parameter_frame, text="Describe Selected Parameter(s)", command=self.describe_selected_parameters)
        self.describe_parameters_button.grid(row=2, column=2, padx=2, pady=2, sticky=tk.W+tk.E)

        self.set_parameter_label = tk.Label(parameter_frame, text="Set Parameter:")
        self.set_parameter_label.grid(row=3, column=0, padx=2, pady=2, sticky=tk.W)
        self.set_parameter_name_label = tk.Label(parameter_frame, text="Name:")
        self.set_parameter_name_label.grid(row=4, column=0, padx=2, pady=2, sticky=tk.W)
        self.set_parameter_name_entry = tk.Entry(parameter_frame)
        self.set_parameter_name_entry.grid(row=4, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        self.set_parameter_value_label = tk.Label(parameter_frame, text="Value:")
        self.set_parameter_value_label.grid(row=5, column=0, padx=2, pady=2, sticky=tk.W)
        self.set_parameter_value_entry = tk.Entry(parameter_frame)
        self.set_parameter_value_entry.grid(row=5, column=1, padx=2, pady=2, sticky=tk.W+tk.E)
        self.set_parameter_button = tk.Button(parameter_frame, text="Set Parameter", command=self.set_single_parameter)
        self.set_parameter_button.grid(row=4, column=2, rowspan=2, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)

        parameter_frame.grid_columnconfigure(0, weight=1)
        parameter_frame.grid_columnconfigure(1, weight=1)
        parameter_frame.grid_columnconfigure(2, weight=1)
        parameter_frame.grid_rowconfigure(1, weight=1) # Make listbox expandable


        # --- Action Frame Widgets ---
        self.action_label = tk.Label(action_frame, text="Rotate Absolute (Selected Turtle):")
        self.action_label.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.action_angle_label = tk.Label(action_frame, text="Target Angle (rad):")
        self.action_angle_label.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W)
        self.action_angle_entry = tk.Entry(action_frame, width=10)
        self.action_angle_entry.grid(row=1, column=1, padx=2, pady=2, sticky=tk.W)
        self.send_goal_button = tk.Button(action_frame, text="Send Goal", command=self.send_rotate_absolute_goal)
        self.send_goal_button.grid(row=1, column=2, padx=2, pady=2, sticky=tk.W+tk.E)
        self.cancel_goal_button = tk.Button(action_frame, text="Cancel Goal", command=self.cancel_rotate_absolute_goal, state=tk.DISABLED)
        self.cancel_goal_button.grid(row=1, column=3, padx=2, pady=2, sticky=tk.W+tk.E)

        self.action_status_label = tk.Label(action_frame, text="Status: Idle")
        self.action_status_label.grid(row=2, column=0, columnspan=4, padx=2, pady=2, sticky=tk.W)
        self.action_feedback_label = tk.Label(action_frame, text="Feedback: ")
        self.action_feedback_label.grid(row=3, column=0, columnspan=4, padx=2, pady=2, sticky=tk.W)
        self.action_result_label = tk.Label(action_frame, text="Result: ")
        self.action_result_label.grid(row=4, column=0, columnspan=4, padx=2, pady=2, sticky=tk.W)

        action_frame.grid_columnconfigure(0, weight=1)
        action_frame.grid_columnconfigure(1, weight=1)
        action_frame.grid_columnconfigure(2, weight=1)
        action_frame.grid_columnconfigure(3, weight=1)


        # --- Info Frame Widgets ---
        self.node_list_label = tk.Label(info_frame, text="ROS Nodes:")
        self.node_list_label.grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.node_listbox = tk.Listbox(info_frame, height=5)
        self.node_listbox.grid(row=1, column=0, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.node_listbox.bind('<<ListboxSelect>>', self.show_node_details)

        self.turtle_list_label = tk.Label(info_frame, text="Active Turtles:")
        self.turtle_list_label.grid(row=2, column=0, padx=2, pady=2, sticky=tk.W)
        self.turtle_listbox = tk.Listbox(info_frame, height=5)
        self.turtle_listbox.grid(row=3, column=0, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.turtle_listbox.bind('<<ListboxSelect>>', self.select_turtle)

        self.topic_list_label = tk.Label(info_frame, text="ROS Topics:")
        self.topic_list_label.grid(row=4, column=0, padx=2, pady=2, sticky=tk.W)
        self.topic_listbox = tk.Listbox(info_frame, height=5)
        self.topic_listbox.grid(row=5, column=0, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.topic_listbox.bind('<<ListboxSelect>>', self.show_topic_details)

        self.service_list_label = tk.Label(info_frame, text="ROS Services:")
        self.service_list_label.grid(row=6, column=0, padx=2, pady=2, sticky=tk.W)
        self.service_listbox = tk.Listbox(info_frame, height=5)
        self.service_listbox.grid(row=7, column=0, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.service_listbox.bind('<<ListboxSelect>>', self.show_service_details)

        self.action_list_label = tk.Label(info_frame, text="ROS Actions:")
        self.action_list_label.grid(row=8, column=0, padx=2, pady=2, sticky=tk.W)
        self.action_listbox = tk.Listbox(info_frame, height=5)
        self.action_listbox.grid(row=9, column=0, padx=2, pady=2, sticky=tk.W+tk.E+tk.N+tk.S)
        self.action_listbox.bind('<<ListboxSelect>>', self.show_action_details)

        self.pose_label = tk.Label(info_frame, text="Selected Turtle Pose:")
        self.pose_label.grid(row=10, column=0, padx=2, pady=2, sticky=tk.W)
        self.pose_text = tk.Label(info_frame, text="X: -, Y: -, Theta: -")
        self.pose_text.grid(row=11, column=0, padx=2, pady=2, sticky=tk.W)

        info_frame.grid_columnconfigure(0, weight=1)
        info_frame.grid_rowconfigure(1, weight=1)
        info_frame.grid_rowconfigure(3, weight=1)
        info_frame.grid_rowconfigure(5, weight=1)
        info_frame.grid_rowconfigure(7, weight=1)
        info_frame.grid_rowconfigure(9, weight=1)


        # --- Details Frame Widgets ---
        self.details_text = scrolledtext.ScrolledText(details_frame, height=10)
        self.details_text.grid(row=0, column=0, padx=5, pady=5, sticky=tk.W+tk.E+tk.N+tk.S)

        details_frame.grid_columnconfigure(0, weight=1)
        details_frame.grid_rowconfigure(0, weight=1)


        # --- Global Update Button ---
        self.update_button = tk.Button(self.window, text="Manual Update All Lists", command=self.update_all_lists)
        self.update_button.grid(row=6, column=0, columnspan=3, padx=5, pady=5, sticky=tk.W+tk.E)


        # Configure main window grid weights
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_columnconfigure(1, weight=1)
        self.window.grid_columnconfigure(2, weight=1)
        self.window.grid_rowconfigure(5, weight=1) # Make details frame expandable


    def on_closing(self):
        """Handle the window closing event to properly shut down ROS2."""
        self.get_logger().info("Shutting down ROS2 and closing GUI.")
        # Destroy all active pose subscribers
        for sub in self.pose_subscribers.values():
            self.destroy_subscription(sub)
        # Destroy all active clients (optional but good practice)
        for client in self.parameter_clients.values():
             self.destroy_client(client)
        for client in self.action_clients.values():
             # Cancel any active action goal before destroying client
             if self.current_action_goal and self.current_action_goal.status in [GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING]:
                 self.current_action_goal.cancel_goal_async()
             self.destroy_action_client(client)


        self.destroy_node()
        rclpy.shutdown()
        self.window.destroy()
        sys.exit(0)

    # --- Turtlesim Control Service Clients ---
    def spawn_turtle(self):
        client = self.create_client(Spawn, '/spawn')
        if not client.wait_for_service(timeout_sec=3.0):
            messagebox.showerror("Service Error", "Spawn service not available.")
            self.get_logger().error('spawn service not available.')
            return

        existing_turtles = self.get_turtle_names()
        base_name = "turtle"
        new_turtle_id = 1
        new_turtle_name = f"{base_name}{new_turtle_id}"
        while new_turtle_name in existing_turtles:
            new_turtle_id += 1
            new_turtle_name = f"{base_name}{new_turtle_id}"

        request = Spawn.Request()
        # Simple way to space out new turtles - can be improved with GUI inputs
        request.x = 5.0 + len(existing_turtles) * 0.7
        request.y = 5.0
        request.theta = 0.0
        request.name = new_turtle_name

        future = client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.log_details(f"Spawned turtle: {result.name}")
                self.update_all_lists()
            else:
                self.log_details("Failed to spawn turtle.")
                messagebox.showerror("Spawn Error", "Failed to spawn turtle.")
        except Exception as e:
            self.log_details(f"Spawn service call failed: {e}")
            messagebox.showerror("Service Error", f"Spawn service call failed: {e}")

    def spawn_turtle_at_position(self):
        """Spawn a turtle at a specific position defined by the user."""
        client = self.create_client(Spawn, '/spawn')
        if not client.wait_for_service(timeout_sec=3.0):
            messagebox.showerror("Service Error", "Spawn service not available.")
            self.get_logger().error('spawn service not available.')
            return

        # Generate a unique turtle name
        existing_turtles = self.get_turtle_names()
        base_name = "turtle"
        new_turtle_id = 1
        new_turtle_name = f"{base_name}{new_turtle_id}"
        while new_turtle_name in existing_turtles:
            new_turtle_id += 1
            new_turtle_name = f"{base_name}{new_turtle_id}"

        # Get position values from entry fields
        try:
            x = float(self.spawn_x_entry.get())
            y = float(self.spawn_y_entry.get())
            theta = float(self.spawn_theta_entry.get())
        except ValueError:
            messagebox.showwarning("Input Error", "Please enter valid float values for X, Y, and Theta.")
            return

        # Create and send the spawn request
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = new_turtle_name

        self.log_details(f"Spawning turtle at X: {x}, Y: {y}, Theta: {theta}")
        future = client.call_async(request)
        future.add_done_callback(self.spawn_callback)


    def kill_turtle(self):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to kill.")
            return

        client = self.create_client(Kill, '/kill')
        if not client.wait_for_service(timeout_sec=3.0):
             messagebox.showerror("Service Error", "Kill service not available.")
             self.get_logger().error('kill service not available.')
             return

        request = Kill.Request()
        request.name = self.selected_turtle
        future = client.call_async(request)
        future.add_done_callback(self.kill_callback)


    def kill_callback(self, future):
        try:
            future.result()
            self.log_details(f"Kill service call completed.")
        except Exception as e:
            self.log_details(f"Kill service call failed: {e}")
            messagebox.showerror("Service Error", f"Kill service call failed: {e}")
        finally:
             self.update_all_lists()
             if self.selected_turtle not in self.get_turtle_names():
                 self.selected_turtle = None
                 self.pose_text.config(text="X: -, Y: -, Theta: -")


    def reset_turtlesim(self):
        client = self.create_client(Empty, '/reset')
        if not client.wait_for_service(timeout_sec=3.0):
             messagebox.showerror("Service Error", "Reset service not available.")
             self.get_logger().error('reset service not available.')
             return

        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(self.reset_callback)

    def reset_callback(self, future):
        try:
            future.result()
            self.log_details("Turtlesim reset completed.")
        except Exception as e:
             self.log_details(f"Reset service call failed: {e}")
             messagebox.showerror("Service Error", f"Reset service call failed: {e}")
        finally:
             self.selected_turtle = None
             self.twist_publishers = {}
             for sub in self.pose_subscribers.values():
                 self.destroy_subscription(sub)
             self.pose_subscribers = {}
             self.current_pose = {}
             self.pose_text.config(text="X: -, Y: -, Theta: -")
             self.update_all_lists()

    # --- Pen Control Service Client ---
    def set_pen(self):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to control the pen.")
            return

        client = self.create_client(SetPen, f'/{self.selected_turtle}/set_pen')
        if not client.wait_for_service(timeout_sec=3.0):
             messagebox.showerror("Service Error", f"SetPen service for {self.selected_turtle} not available.")
             self.get_logger().error(f'SetPen service for {self.selected_turtle} not available.')
             return

        request = SetPen.Request()
        try:
            request.r = int(self.pen_r_entry.get())
            request.g = int(self.pen_g_entry.get())
            request.b = int(self.pen_b_entry.get())
            request.width = int(self.pen_width_entry.get())
            request.off = self.pen_off_var.get()
        except ValueError:
            messagebox.showwarning("Input Error", "Please enter valid integer values for pen color and width.")
            return

        future = client.call_async(request)
        future.add_done_callback(lambda future: self.log_service_callback_result(future, 'SetPen'))

    # --- Teleport Service Clients ---
    def teleport_absolute(self):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to teleport.")
            return

        client = self.create_client(TeleportAbsolute, f'/{self.selected_turtle}/teleport_absolute')
        if not client.wait_for_service(timeout_sec=3.0):
             messagebox.showerror("Service Error", f"TeleportAbsolute service for {self.selected_turtle} not available.")
             self.get_logger().error(f'TeleportAbsolute service for {self.selected_turtle} not available.')
             return

        request = TeleportAbsolute.Request()
        try:
            request.x = float(self.teleport_abs_x_entry.get())
            request.y = float(self.teleport_abs_y_entry.get())
            request.theta = float(self.teleport_abs_theta_entry.get())
        except ValueError:
            messagebox.showwarning("Input Error", "Please enter valid float values for position and orientation.")
            return

        future = client.call_async(request)
        future.add_done_callback(lambda future: self.log_service_callback_result(future, 'TeleportAbsolute'))

    def teleport_relative(self):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to teleport.")
            return

        client = self.create_client(TeleportRelative, f'/{self.selected_turtle}/teleport_relative')
        if not client.wait_for_service(timeout_sec=3.0):
             messagebox.showerror("Service Error", f"TeleportRelative service for {self.selected_turtle} not available.")
             self.get_logger().error(f'TeleportRelative service for {self.selected_turtle} not available.')
             return

        request = TeleportRelative.Request()
        try:
            request.linear = float(self.teleport_rel_linear_entry.get())
            request.angular = float(self.teleport_rel_angular_entry.get())
        except ValueError:
            messagebox.showwarning("Input Error", "Please enter valid float values for linear and angular distance.")
            return

        future = client.call_async(request)
        future.add_done_callback(lambda future: self.log_service_callback_result(future, 'TeleportRelative'))

    # --- Parameter Service Clients ---
    def get_parameter_client(self, service_name):
        """Helper to get or create a parameter service client."""
        if service_name not in self.parameter_clients:
            if service_name == '/turtlesim/describe_parameters':
                self.parameter_clients[service_name] = self.create_client(DescribeParameters, service_name)
            elif service_name == '/turtlesim/get_parameter_types':
                 self.parameter_clients[service_name] = self.create_client(GetParameterTypes, service_name)
            elif service_name == '/turtlesim/get_parameters':
                 self.parameter_clients[service_name] = self.create_client(GetParameters, service_name)
            elif service_name == '/turtlesim/list_parameters':
                 self.parameter_clients[service_name] = self.create_client(ListParameters, service_name)
            elif service_name == '/turtlesim/set_parameters':
                 self.parameter_clients[service_name] = self.create_client(SetParameters, service_name)
            elif service_name == '/turtlesim/set_parameters_atomically':
                 self.parameter_clients[service_name] = self.create_client(SetParametersAtomically, service_name)
            else:
                self.get_logger().error(f"Unknown parameter service: {service_name}")
                return None
            self.get_logger().info(f"Created client for {service_name}")
        return self.parameter_clients[service_name]


    def list_parameters(self):
        client = self.get_parameter_client('/turtlesim/list_parameters')
        if client is None or not client.wait_for_service(timeout_sec=3.0):
            messagebox.showerror("Service Error", "ListParameters service not available.")
            self.get_logger().error('ListParameters service not available.')
            return

        request = ListParameters.Request()
        future = client.call_async(request)
        future.add_done_callback(self.list_parameters_callback)

    def list_parameters_periodic(self):
         # Called by timer, no need for wait_for_service or error box
         client = self.get_parameter_client('/turtlesim/list_parameters')
         if client is not None and client.service_is_ready():
             request = ListParameters.Request()
             future = client.call_async(request)
             future.add_done_callback(self.list_parameters_callback)


    def list_parameters_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.parameter_list = result.result.names
                # Update the parameter listbox in the GUI thread
                self.window.after(0, self._update_parameter_listbox)
                self.log_details("Parameters listed.")
            else:
                self.log_details("Failed to list parameters.")
        except Exception as e:
            self.log_details(f"ListParameters service call failed: {e}")

    def _update_parameter_listbox(self):
        """Safely updates the parameter listbox in the main Tkinter thread."""
        self.parameter_listbox.delete(0, tk.END)
        for param_name in self.parameter_list:
            self.parameter_listbox.insert(tk.END, param_name)


    def get_selected_parameters(self):
        selected_indices = self.parameter_listbox.curselection()
        if not selected_indices:
            messagebox.showwarning("Warning", "Please select one or more parameters.")
            return

        param_names = [self.parameter_listbox.get(i) for i in selected_indices]

        client = self.get_parameter_client('/turtlesim/get_parameters')
        if client is None or not client.wait_for_service(timeout_sec=3.0):
            messagebox.showerror("Service Error", "GetParameters service not available.")
            self.get_logger().error('GetParameters service not available.')
            return

        request = GetParameters.Request()
        request.names = param_names
        future = client.call_async(request)
        future.add_done_callback(self.get_parameters_callback)

    def get_parameters_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.log_details("--- Get Parameters Result ---")
                for param in result.values:
                    # ParameterValue message structure: type, bool_value, integer_value, double_value, string_value, byte_array_value, bool_array_value, integer_array_value, double_array_value, string_array_value
                    # Need to determine the type to get the correct value
                    param_value_str = "N/A"
                    if param.type == 1: # BOOL
                        param_value_str = str(param.bool_value)
                    elif param.type == 2: # INTEGER
                        param_value_str = str(param.integer_value)
                    elif param.type == 3: # DOUBLE
                        param_value_str = str(param.double_value)
                    elif param.type == 4: # STRING
                        param_value_str = param.string_value
                    # Add more types if needed

                    # Find the parameter name from the original request (not directly available in response)
                    # A more robust way would involve storing the request with the future.
                    # For simplicity, we'll just list the values received.
                    self.log_details(f"Value: {param_value_str} (Type: {param.type})")
                self.log_details("-----------------------------")
            else:
                self.log_details("Failed to get parameters.")
        except Exception as e:
            self.log_details(f"GetParameters service call failed: {e}")


    def describe_selected_parameters(self):
        selected_indices = self.parameter_listbox.curselection()
        if not selected_indices:
            messagebox.showwarning("Warning", "Please select one or more parameters.")
            return

        param_names = [self.parameter_listbox.get(i) for i in selected_indices]

        client = self.get_parameter_client('/turtlesim/describe_parameters')
        if client is None or not client.wait_for_service(timeout_sec=3.0):
            messagebox.showerror("Service Error", "DescribeParameters service not available.")
            self.get_logger().error('DescribeParameters service not available.')
            return

        request = DescribeParameters.Request()
        request.names = param_names
        future = client.call_async(request)
        future.add_done_callback(self.describe_parameters_callback)

    def describe_parameters_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.log_details("--- Describe Parameters Result ---")
                for descriptor in result.descriptors:
                    # ParameterDescriptor message structure: name, type, description, additional_constraints, read_only, dynamic_typing, floating_point_range, integer_range
                    self.log_details(f"Name: {descriptor.name}")
                    self.log_details(f"  Type: {descriptor.type}")
                    self.log_details(f"  Description: {descriptor.description}")
                    self.log_details(f"  Read Only: {descriptor.read_only}")
                    # Add more details from descriptor as needed
                self.log_details("----------------------------------")
            else:
                self.log_details("Failed to describe parameters.")
        except Exception as e:
            self.log_details(f"DescribeParameters service call failed: {e}")


    def set_single_parameter(self):
        param_name = self.set_parameter_name_entry.get()
        param_value_str = self.set_parameter_value_entry.get()

        self.log_details(f"Attempting to set parameter: {param_name} = {param_value_str}")

        if not param_name:
            messagebox.showwarning("Input Error", "Please enter a parameter name.")
            return
        if not param_value_str:
             messagebox.showwarning("Input Error", "Please enter a parameter value.")
             return

        # Attempt to infer parameter type (basic inference)
        param_type = None # This will be rclpy.parameter.Parameter.Type enum member
        param_value = None # This will be the Python primitive value

        try:
            param_value = int(param_value_str)
            param_type = rclpy.parameter.Parameter.Type.INTEGER
            self.log_details(f"Detected parameter type: INTEGER ({param_type})")
        except ValueError:
            try:
                param_value = float(param_value_str)
                param_type = rclpy.parameter.Parameter.Type.DOUBLE
                self.log_details(f"Detected parameter type: DOUBLE ({param_type})")
            except ValueError:
                if param_value_str.lower() in ['true', 'false']:
                    param_value = param_value_str.lower() == 'true'
                    param_type = rclpy.parameter.Parameter.Type.BOOL
                    self.log_details(f"Detected parameter type: BOOL ({param_type})")
                else:
                    param_value = param_value_str
                    param_type = rclpy.parameter.Parameter.Type.STRING
                    self.log_details(f"Detected parameter type: STRING ({param_type})")
                    # Add handling for lists/arrays if needed

        # For background color parameters, ensure they are in the valid range (0-255)
        if param_name in ['background_r', 'background_g', 'background_b'] and param_type == rclpy.parameter.Parameter.Type.INTEGER: # Compare with enum
            if param_value < 0 or param_value > 255:
                messagebox.showwarning("Input Error", f"Color value must be between 0 and 255. Got {param_value}.")
                return

        self.log_details(f"Creating parameter client for /turtlesim/set_parameters")
        client = self.get_parameter_client('/turtlesim/set_parameters')
        if client is None:
            self.log_details("Failed to create parameter client")
            messagebox.showerror("Service Error", "Failed to create SetParameters client.")
            return

        self.log_details(f"Waiting for service...")
        if not client.wait_for_service(timeout_sec=3.0):
            self.log_details("Service not available after timeout")
            messagebox.showerror("Service Error", "SetParameters service not available.")
            self.get_logger().error('SetParameters service not available.')
            return

        self.log_details(f"Creating parameter message manually")
        request = SetParameters.Request()
        try:
            # Manual construction of rcl_interfaces.msg.Parameter
            pv_msg = ParameterValueMsg()
            pv_msg.type = param_type.value # Get integer from enum member

            if param_type == rclpy.parameter.Parameter.Type.BOOL:
                pv_msg.bool_value = param_value
            elif param_type == rclpy.parameter.Parameter.Type.INTEGER:
                pv_msg.integer_value = param_value
            elif param_type == rclpy.parameter.Parameter.Type.DOUBLE:
                pv_msg.double_value = param_value
            elif param_type == rclpy.parameter.Parameter.Type.STRING:
                pv_msg.string_value = param_value
            # Add other types here if supported by the GUI's inference logic
            else:
                # Fallback or error for unhandled inferred types
                self.log_details(f"Error: Unhandled parameter type ({param_type}) for manual message creation.")
                messagebox.showerror("Parameter Error", f"Unhandled parameter type: {param_type}")
                return

            parameter_to_set = ParameterMsg()
            parameter_to_set.name = param_name
            parameter_to_set.value = pv_msg
            
            request.parameters.append(parameter_to_set)

        except Exception as e:
            self.log_details(f"Error creating parameter message: {e}")
            messagebox.showerror("Parameter Error", f"Error creating parameter: {e}")
            return

        self.log_details(f"Sending parameter request: {param_name}={param_value} (type name={param_type.name}, type value={param_type.value})")
        future = client.call_async(request)
        future.add_done_callback(self.set_parameters_callback)

    def set_parameters_callback(self, future):
        try:
            self.log_details("Parameter service call completed, checking result...")
            result = future.result()
            if result is not None and hasattr(result, 'results') and result.results:
                self.log_details("--- Set Parameters Result ---")
                for res in result.results:
                    self.log_details(f"  Successful: {res.successful}")
                    self.log_details(f"  Reason: {res.reason if res.reason else 'No reason provided'}")

                    if not res.successful:
                        messagebox.showwarning("Parameter Warning", f"Failed to set parameter: {res.reason if res.reason else 'Unknown error'}")
                self.log_details("-----------------------------")
            else:
                self.log_details("Failed to set parameters: No valid result returned")
                messagebox.showerror("Parameter Error", "Failed to set parameter: No valid result returned")
        except Exception as e:
            self.log_details(f"SetParameters service call failed with exception: {e}")
            messagebox.showerror("Service Error", f"SetParameters service call failed: {e}")

    # --- Action Client (RotateAbsolute) ---
    def get_action_client(self, action_name, action_type):
        """Helper to get or create an action client."""
        if action_name not in self.action_clients:
            self.action_clients[action_name] = ActionClient(self, action_type, action_name)
            self.get_logger().info(f"Created action client for {action_name}")
        return self.action_clients[action_name]


    def send_rotate_absolute_goal(self):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to send an action goal.")
            return

        action_name = f'/{self.selected_turtle}/rotate_absolute'
        client = self.get_action_client(action_name, RotateAbsolute)

        if not client.wait_for_server(timeout_sec=5.0):
            messagebox.showerror("Action Error", f"Action server for {action_name} not available.")
            self.get_logger().error(f"Action server for {action_name} not available.")
            return

        goal_msg = RotateAbsolute.Goal()
        try:
            goal_msg.theta = float(self.action_angle_entry.get())
        except ValueError:
            messagebox.showwarning("Input Error", "Please enter a valid float value for the target angle.")
            return

        self.log_details(f"Sending goal to {action_name}: {goal_msg.theta} radians")
        self.action_status_label.config(text="Status: Sending Goal...")
        self.cancel_goal_button.config(state=tk.DISABLED) # Disable cancel until goal is accepted

        # Send the goal and register callbacks
        self._send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.rotate_absolute_feedback_callback)
        self._send_goal_future.add_done_callback(self.rotate_absolute_goal_response_callback)


    def rotate_absolute_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle:
            self.log_details("Goal was rejected by the action server.")
            self.action_status_label.config(text="Status: Goal Rejected")
            self.cancel_goal_button.config(state=tk.DISABLED)
            return

        self.log_details("Goal accepted by the action server.")
        self.action_status_label.config(text="Status: Goal Accepted")
        self.cancel_goal_button.config(state=tk.NORMAL) # Enable cancel button
        self.current_action_goal = goal_handle # Store the goal handle

        # Wait for the server to be done with the goal
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.rotate_absolute_get_result_callback)


    def rotate_absolute_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Update feedback display in the GUI thread
        self.window.after(0, lambda: self.action_feedback_label.config(text=f"Feedback: Remaining: {feedback.remaining:.2f}"))


    def rotate_absolute_get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.current_action_goal = None # Clear the stored goal handle

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.log_details(f"Goal succeeded! Result: {result.delta:.2f} radians")
            self.action_status_label.config(text="Status: Succeeded")
            self.action_result_label.config(text=f"Result: Delta: {result.delta:.2f}")
        else:
            self.log_details(f"Goal failed with status: {status}")
            self.action_status_label.config(text=f"Status: Failed ({status})")
            self.action_result_label.config(text="Result: N/A")

        self.cancel_goal_button.config(state=tk.DISABLED) # Disable cancel after goal is done
        self.action_feedback_label.config(text="Feedback: ") # Clear feedback


    def cancel_rotate_absolute_goal(self):
        if self.current_action_goal:
            self.log_details("Canceling active goal...")
            self.action_status_label.config(text="Status: Canceling...")
            cancel_future = self.current_action_goal.cancel_goal_async()
            cancel_future.add_done_callback(self.rotate_absolute_cancel_response_callback)
        else:
            self.log_details("No active goal to cancel.")


    def rotate_absolute_cancel_response_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.log_details("Goal successfully canceled.")
        else:
            self.log_details("Goal cancellation failed.")


    # --- ROS Information and Details ---
    def get_turtle_names(self):
        """
        Finds active turtle names by listing topics and filtering for /turtleX/pose.
        """
        topic_names_and_types = self.get_topic_names_and_types()
        turtle_names = []
        for topic_name, topic_types in topic_names_and_types:
            match = re.match(r'/(\w+)/pose', topic_name)
            if match:
                turtle_name = match.group(1)
                if turtle_name not in turtle_names:
                    turtle_names.append(turtle_name)
        return turtle_names


    def update_all_lists(self):
        # This function is called by the manual update button or after service calls.
        # It triggers updates for all lists and parameters.
        # The actual GUI updates happen via window.after in the individual update methods.
        self.update_lists() # Updates nodes, turtles, topics, services, actions lists
        self.list_parameters() # Triggers parameter list update


    def update_lists(self):
        # This function is called by the ROS2 timer, running in the executor thread.
        # Tkinter listbox updates must happen in the main thread.
        # Use window.after to safely update the GUI from a different thread.
        def _update_gui_lists():
            # Clear current lists
            self.node_listbox.delete(0, tk.END)
            self.turtle_listbox.delete(0, tk.END)
            self.topic_listbox.delete(0, tk.END)
            self.service_listbox.delete(0, tk.END)
            self.action_listbox.delete(0, tk.END)
            # Don't clear details_text here, as it might contain user-requested info

            # Get and display nodes
            nodes = self.get_node_names()
            for node in nodes:
                self.node_listbox.insert(tk.END, node)

            # Get and display active turtles and manage pose subscribers
            turtle_names = self.get_turtle_names()
            active_subscribers = list(self.pose_subscribers.keys())

            for turtle_name in turtle_names:
                self.turtle_listbox.insert(tk.END, turtle_name)
                # If a turtle is active but we don't have a subscriber for it, create one
                if turtle_name not in active_subscribers:
                     topic_name = f'/{turtle_name}/pose'
                     self.pose_subscribers[turtle_name] = self.create_subscription(
                         Pose,
                         topic_name,
                         lambda msg, name=turtle_name: self.pose_callback(msg, name),
                         10
                     )
                     self.get_logger().info(f"Created pose subscriber for {topic_name}")
                     self.current_pose[turtle_name] = None


            # Remove subscribers for turtles that are no longer active
            for sub_name in active_subscribers:
                if sub_name not in turtle_names:
                    self.destroy_subscription(self.pose_subscribers[sub_name])
                    del self.pose_subscribers[sub_name]
                    if sub_name in self.current_pose:
                         del self.current_pose[sub_name]
                    self.get_logger().info(f"Destroyed pose subscriber for /{sub_name}/pose")


            # Get and display topics
            topics = self.get_topic_names_and_types()
            for topic_name, _ in topics:
                self.topic_listbox.insert(tk.END, topic_name)

            # Get and display services
            services = self.get_service_names_and_types()
            for service_name, _ in services:
                self.service_listbox.insert(tk.END, service_name)

            # Get and display actions (Note: ROS 2 doesn't have a direct API to list actions easily)
            # We will list known turtlesim actions here.
            actions = []
            for turtle_name in turtle_names:
                 actions.append(f"/{turtle_name}/rotate_absolute") # Add rotate_absolute for each turtle

            # Add turtlesim-specific actions if they exist and are not tied to a specific turtle name pattern
            # actions.append("/some_global_turtlesim_action") # Example

            # Remove duplicates and sort
            actions = sorted(list(set(actions)))

            for action in actions:
                self.action_listbox.insert(tk.END, action)


        # Schedule the GUI update function to run in the main Tkinter thread
        self.window.after(0, _update_gui_lists)


    def pose_callback(self, msg, turtle_name):
        """Callback function for turtle pose updates."""
        self.current_pose[turtle_name] = msg
        if self.selected_turtle == turtle_name:
            self.window.after(0, self._update_pose_display)


    def _update_pose_display(self):
        """Safely updates the pose display in the main Tkinter thread."""
        if self.selected_turtle and self.selected_turtle in self.current_pose and self.current_pose[self.selected_turtle]:
             pose = self.current_pose[self.selected_turtle]
             theta_degrees = math.degrees(pose.theta)
             self.pose_text.config(text=f"X: {pose.x:.2f}, Y: {pose.y:.2f}, Theta: {theta_degrees:.2f}°")
        else:
             self.pose_text.config(text="X: -, Y: -, Theta: -")


    def select_turtle(self, event):
        selected_indices = self.turtle_listbox.curselection()
        if selected_indices:
            self.selected_turtle = self.turtle_listbox.get(selected_indices[0])
            self.log_details(f"Selected turtle: {self.selected_turtle}")
            self._update_pose_display()
            # Clear action status/feedback/result for the previous turtle
            self.action_status_label.config(text="Status: Idle")
            self.action_feedback_label.config(text="Feedback: ")
            self.action_result_label.config(text="Result: ")
            self.cancel_goal_button.config(state=tk.DISABLED)
            self.current_action_goal = None # Clear any active goal handle

        else:
            self.selected_turtle = None
            self.log_details("No turtle selected.")
            self.pose_text.config(text="X: -, Y: -, Theta: -")
            self.action_status_label.config(text="Status: Idle")
            self.action_feedback_label.config(text="Feedback: ")
            self.action_result_label.config(text="Result: ")
            self.cancel_goal_button.config(state=tk.DISABLED)
            self.current_action_goal = None


    def publish_twist(self, linear_x, angular_z):
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to move.")
            return

        if self.selected_turtle not in self.twist_publishers:
            topic_name = f'/{self.selected_turtle}/cmd_vel'
            self.twist_publishers[self.selected_turtle] = self.create_publisher(Twist, topic_name, 10)
            self.get_logger().info(f"Created publisher for {topic_name}")


        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.twist_publishers[self.selected_turtle].publish(twist)
        self.get_logger().info(f"Publishing Twist to /{self.selected_turtle}/cmd_vel: Linear X={linear_x}, Angular Z={angular_z}")


    def show_node_details(self, event):
        selected_indices = self.node_listbox.curselection()
        if selected_indices:
            selected_node = self.node_listbox.get(selected_indices[0])
            self.log_details(f"Details for Node: {selected_node}\n")
            # More detailed introspection would require additional ROS2 APIs or tools
        else:
             self.details_text.delete("1.0", tk.END)


    def show_topic_details(self, event):
        selected_indices = self.topic_listbox.curselection()
        if selected_indices:
            selected_topic = self.topic_listbox.get(selected_indices[0])
            self.log_details(f"Details for Topic: {selected_topic}\n")
            topics = self.get_topic_names_and_types()
            for topic_name, topic_type in topics:
                if topic_name == selected_topic:
                    self.log_details(f"  Type: {topic_type}\n")
                    # More detailed introspection would require additional ROS2 APIs or tools
                    break
        else:
            self.details_text.delete("1.0", tk.END)


    def show_service_details(self, event):
        selected_indices = self.service_listbox.curselection()
        if selected_indices:
            selected_service = self.service_listbox.get(selected_indices[0])
            self.log_details(f"Details for Service: {selected_service}\n")
            services = self.get_service_names_and_types()
            for service_name, service_type in services:
                if service_name == selected_service:
                    self.log_details(f"  Type: {service_type}\n")
                    # More detailed introspection would require additional ROS2 APIs or tools
                    break
        else:
            self.details_text.delete("1.0", tk.END)


    def show_action_details(self, event):
        selected_indices = self.action_listbox.curselection()
        if selected_indices:
            selected_action = self.action_listbox.get(selected_indices[0])
            self.log_details(f"Details for Action: {selected_action}\n")
            # More detailed introspection would require additional ROS2 APIs or tools
        else:
            self.details_text.delete("1.0", tk.END)

    def show_parameter_details(self, event):
         selected_indices = self.parameter_listbox.curselection()
         if selected_indices:
             selected_param = self.parameter_listbox.get(selected_indices[0])
             self.log_details(f"Selected Parameter: {selected_param}\n")
             # You can add logic here to automatically call get_parameters or describe_parameters
             # for the selected parameter if you want more details to appear on selection.
         else:
             pass # Do nothing if no parameter is selected


    def log_details(self, message):
        """Safely logs messages to the details text area in the main Tkinter thread."""
        self.window.after(0, lambda: self._insert_details_text(message + "\n"))

    def _insert_details_text(self, text):
        """Inserts text into the details text area."""
        self.details_text.insert(tk.END, text)
        self.details_text.see(tk.END) # Auto-scroll to the bottom


    def log_service_callback_result(self, future, service_name):
        """Generic callback to log the result of a service call."""
        try:
            result = future.result()
            if result is not None:
                self.log_details(f"{service_name} service call successful.")
                # You could add logic here to print specific result details if the service has them
            else:
                 self.log_details(f"{service_name} service call failed (no result).")
        except Exception as e:
            self.log_details(f"{service_name} service call failed: {e}")

    # --- Movement Control Methods ---
    def move_forward(self):
        """Move the selected turtle forward."""
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to move.")
            return
        self.publish_twist(0.5, 0.0)  # Linear velocity 0.5, angular velocity 0

    def move_backward(self):
        """Move the selected turtle backward."""
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to move.")
            return
        self.publish_twist(-0.5, 0.0)  # Linear velocity -0.5, angular velocity 0

    def turn_left(self):
        """Turn the selected turtle left."""
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to move.")
            return
        self.publish_twist(0.0, 0.5)  # Linear velocity 0, angular velocity 0.5

    def turn_right(self):
        """Turn the selected turtle right."""
        if not self.selected_turtle:
            messagebox.showwarning("Warning", "Please select a turtle to move.")
            return
        self.publish_twist(0.0, -0.5)  # Linear velocity 0, angular velocity -0.5

    def publish_twist(self, linear_x, angular_z):
        """Publish a Twist message to move the selected turtle."""
        if not self.selected_turtle:
            return

        # Create a publisher if it doesn't exist for this turtle
        if self.selected_turtle not in self.twist_publishers:
            topic_name = f'/{self.selected_turtle}/cmd_vel'
            self.twist_publishers[self.selected_turtle] = self.create_publisher(
                Twist,
                topic_name,
                10
            )
            self.get_logger().info(f"Created twist publisher for {topic_name}")

        # Create and publish the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(angular_z)
        self.twist_publishers[self.selected_turtle].publish(twist_msg)
        self.log_details(f"Published Twist: linear.x={linear_x}, angular.z={angular_z}")


    def run(self):
        # Initial update of lists
        self.update_all_lists()
        # The Tkinter main loop runs in the main thread
        self.window.mainloop()


# Function to run the ROS2 spinning in a separate thread
def spin_ros_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    gui = TurtlesimGUI()

    # Create and start a separate thread for ROS2 spinning
    ros_thread = threading.Thread(target=spin_ros_node, args=(gui,), daemon=True) # Use daemon=True
    ros_thread.start()

    # Run the Tkinter GUI main loop in the main thread
    gui.run()

    # The daemon thread will exit automatically when the main thread exits.
    # No need to explicitly join() if daemon=True.
    # ros_thread.join()


if __name__ == '__main__':
    main()

