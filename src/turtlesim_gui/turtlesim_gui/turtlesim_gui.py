import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from turtlesim.srv import Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose # Import Pose message type
from std_srvs.srv import Empty
import re # Import the regular expression module
import threading # Import the threading module
import sys # Import sys for exit
import math # Import math for converting radians to degrees

class TurtlesimGUI(Node):
    def __init__(self):
        super().__init__('turtlesim_gui')
        self.window = tk.Tk()
        self.window.title("Turtlesim GUI")
        # Set the protocol for closing the window
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.selected_turtle = None # Variable to store the selected turtle name
        self.twist_publishers = {} # Dictionary to hold publishers for each turtle
        self.pose_subscribers = {} # Dictionary to hold pose subscribers for each turtle
        self.current_pose = {} # Dictionary to store the latest pose for each turtle


        self.create_widgets()
        # Timer to update lists every 2 seconds. This timer's callback runs in the ROS2 executor thread.
        self.update_timer = self.create_timer(2.0, self.update_lists)


    def create_widgets(self):
        # --- ROS Information Display ---
        self.node_list_label = tk.Label(self.window, text="ROS Nodes:")
        self.node_list_label.grid(row=0, column=0, sticky=tk.W)
        self.node_listbox = tk.Listbox(self.window, height=5)
        self.node_listbox.grid(row=1, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.node_listbox.bind('<<ListboxSelect>>', self.show_node_details) # Bind selection event

        self.turtle_list_label = tk.Label(self.window, text="Active Turtles:")
        self.turtle_list_label.grid(row=2, column=0, sticky=tk.W)
        self.turtle_listbox = tk.Listbox(self.window, height=5)
        self.turtle_listbox.grid(row=3, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.turtle_listbox.bind('<<ListboxSelect>>', self.select_turtle) # Bind selection event

        self.topic_list_label = tk.Label(self.window, text="ROS Topics:")
        self.topic_list_label.grid(row=4, column=0, sticky=tk.W)
        self.topic_listbox = tk.Listbox(self.window, height=5)
        self.topic_listbox.grid(row=5, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.topic_listbox.bind('<<ListboxSelect>>', self.show_topic_details) # Bind selection event

        self.service_list_label = tk.Label(self.window, text="ROS Services:")
        self.service_list_label.grid(row=6, column=0, sticky=tk.W)
        self.service_listbox = tk.Listbox(self.window, height=5)
        self.service_listbox.grid(row=7, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.service_listbox.bind('<<ListboxSelect>>', self.show_service_details) # Bind selection event

        self.action_list_label = tk.Label(self.window, text="ROS Actions:")
        self.action_list_label.grid(row=8, column=0, sticky=tk.W)
        self.action_listbox = tk.Listbox(self.window, height=5)
        self.action_listbox.grid(row=9, column=0, columnspan=2, sticky=tk.W+tk.E)
        self.action_listbox.bind('<<ListboxSelect>>', self.show_action_details) # Bind selection event

        # --- Turtlesim Control Buttons ---
        self.spawn_button = tk.Button(self.window, text="Spawn Turtle", command=self.spawn_turtle)
        self.spawn_button.grid(row=10, column=0, sticky=tk.W+tk.E)
        self.kill_button = tk.Button(self.window, text="Kill Turtle", command=self.kill_turtle)
        self.kill_button.grid(row=10, column=1, sticky=tk.W+tk.E)
        self.reset_button = tk.Button(self.window, text="Reset Turtlesim", command=self.reset_turtlesim)
        self.reset_button.grid(row=11, column=0, columnspan=2, sticky=tk.W+tk.E)

        # --- Turtle Movement Controls ---
        self.movement_label = tk.Label(self.window, text="Move Selected Turtle:")
        self.movement_label.grid(row=12, column=0, columnspan=2, sticky=tk.S)

        self.forward_button = tk.Button(self.window, text="Forward", command=self.move_forward)
        self.forward_button.grid(row=13, column=0, sticky=tk.W+tk.E)
        self.backward_button = tk.Button(self.window, text="Backward", command=self.move_backward)
        self.backward_button.grid(row=13, column=1, sticky=tk.W+tk.E)
        self.left_button = tk.Button(self.window, text="Left", command=self.turn_left)
        self.left_button.grid(row=14, column=0, sticky=tk.W+tk.E)
        self.right_button = tk.Button(self.window, text="Right", command=self.turn_right)
        self.right_button.grid(row=14, column=1, sticky=tk.W+tk.E)

        # --- Text Area for Details ---
        self.details_label = tk.Label(self.window, text="Details:")
        self.details_label.grid(row=0, column=2, sticky=tk.W)
        self.details_text = scrolledtext.ScrolledText(self.window, height=15, width=50) # Adjusted height
        self.details_text.grid(row=1, column=2, rowspan=10, sticky=tk.W+tk.E+tk.N+tk.S) # Adjusted rowspan

        # --- Selected Turtle Pose Display ---
        self.pose_label = tk.Label(self.window, text="Selected Turtle Pose:")
        self.pose_label.grid(row=11, column=2, sticky=tk.W)
        self.pose_text = tk.Label(self.window, text="X: -, Y: -, Theta: -") # Label to display pose
        self.pose_text.grid(row=12, column=2, sticky=tk.W)


        # --- Update Button ---
        self.update_button = tk.Button(self.window, text="Manual Update", command=self.update_lists)
        self.update_button.grid(row=15, column=0, columnspan=3, sticky=tk.W+tk.E) # Adjusted row

        # Configure grid weights for resizing
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_columnconfigure(1, weight=1)
        self.window.grid_columnconfigure(2, weight=2)
        self.window.grid_rowconfigure(1, weight=1)
        self.window.grid_rowconfigure(3, weight=1)
        self.window.grid_rowconfigure(5, weight=1)
        self.window.grid_rowconfigure(7, weight=1)
        self.window.grid_rowconfigure(9, weight=1)
        self.window.grid_rowconfigure(10, weight=0) # Give less weight to button rows
        self.window.grid_rowconfigure(11, weight=0)
        self.window.grid_rowconfigure(12, weight=0)
        self.window.grid_rowconfigure(13, weight=0)
        self.window.grid_rowconfigure(14, weight=0)
        self.window.grid_rowconfigure(15, weight=0)


    def on_closing(self):
        """Handle the window closing event to properly shut down ROS2."""
        self.get_logger().info("Shutting down ROS2 and closing GUI.")
        # Destroy all active pose subscribers
        for sub in self.pose_subscribers.values():
            self.destroy_subscription(sub)
        self.destroy_node()
        rclpy.shutdown()
        self.window.destroy()
        sys.exit(0) # Exit the application

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
        request.x = 5.0 + len(existing_turtles) * 0.5
        request.y = 5.0
        request.theta = 0.0
        request.name = new_turtle_name

        future = client.call_async(request)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f"Spawned turtle: {result.name}")
                self.update_lists()
            else:
                self.get_logger().error("Failed to spawn turtle.")
                messagebox.showerror("Spawn Error", "Failed to spawn turtle.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            messagebox.showerror("Service Error", f"Spawn service call failed: {e}")


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
            self.get_logger().info(f"Kill service call completed.")
        except Exception as e:
            self.get_logger().error(f"Kill service call failed: {e}")
            messagebox.showerror("Service Error", f"Kill service call failed: {e}")
        finally:
             # Unsubscribe from the killed turtle's pose topic
             killed_turtle_name = None # Need to find a way to get the name from the completed future if not stored earlier
             # For now, rely on update_lists to clean up
             self.update_lists()
             if self.selected_turtle not in self.get_turtle_names():
                 self.selected_turtle = None
                 self.pose_text.config(text="X: -, Y: -, Theta: -") # Clear pose display


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
            self.get_logger().info("Turtlesim reset completed.")
        except Exception as e:
             self.get_logger().error(f"Reset service call failed: {e}")
             messagebox.showerror("Service Error", f"Reset service call failed: {e}")
        finally:
             self.selected_turtle = None
             self.twist_publishers = {}
             # Destroy all pose subscribers as turtles are reset
             for sub in self.pose_subscribers.values():
                 self.destroy_subscription(sub)
             self.pose_subscribers = {}
             self.current_pose = {} # Clear stored poses
             self.pose_text.config(text="X: -, Y: -, Theta: -") # Clear pose display
             self.update_lists()


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
                         lambda msg, name=turtle_name: self.pose_callback(msg, name), # Use lambda to pass turtle name
                         10
                     )
                     self.get_logger().info(f"Created pose subscriber for {topic_name}")
                     self.current_pose[turtle_name] = None # Initialize pose entry


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

            # Get and display actions (Note: ROS 2 doesn't have a direct CLI command to list actions)
            # You might need to explore action introspection or pre-define known actions.
            actions = ["/turtle1/rotate_absolute"]  # Example action - Replace with actual actions if available
            for action in actions:
                self.action_listbox.insert(tk.END, action)


        # Schedule the GUI update function to run in the main Tkinter thread
        self.window.after(0, _update_gui_lists)


    def pose_callback(self, msg, turtle_name):
        """Callback function for turtle pose updates."""
        # This callback runs in the ROS2 executor thread.
        self.current_pose[turtle_name] = msg
        # Update the pose display if this turtle is the selected one
        if self.selected_turtle == turtle_name:
            # Schedule GUI update in the main thread
            self.window.after(0, self._update_pose_display)


    def _update_pose_display(self):
        """Safely updates the pose display in the main Tkinter thread."""
        if self.selected_turtle and self.selected_turtle in self.current_pose and self.current_pose[self.selected_turtle]:
             pose = self.current_pose[self.selected_turtle]
             # Convert theta from radians to degrees for display
             theta_degrees = math.degrees(pose.theta)
             self.pose_text.config(text=f"X: {pose.x:.2f}, Y: {pose.y:.2f}, Theta: {theta_degrees:.2f}°")
        else:
             self.pose_text.config(text="X: -, Y: -, Theta: -")


    def select_turtle(self, event):
        selected_indices = self.turtle_listbox.curselection()
        if selected_indices:
            self.selected_turtle = self.turtle_listbox.get(selected_indices[0])
            # Clear details and show selection
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert(tk.END, f"Selected turtle: {self.selected_turtle}\n")
            self.get_logger().info(f"Selected turtle: {self.selected_turtle}")
            # Update pose display for the newly selected turtle
            self._update_pose_display()
        else:
            self.selected_turtle = None
            self.details_text.delete("1.0", tk.END) # Clear details when no turtle is selected
            self.details_text.insert(tk.END, "No turtle selected.\n")
            self.get_logger().info("No turtle selected.")
            self.pose_text.config(text="X: -, Y: -, Theta: -") # Clear pose display


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


    def move_forward(self):
        self.publish_twist(1.0, 0.0)

    def move_backward(self):
        self.publish_twist(-1.0, 0.0)

    def turn_left(self):
        self.publish_twist(0.0, 1.0)

    def turn_right(self):
        self.publish_twist(0.0, -1.0)


    def show_node_details(self, event):
        selected_indices = self.node_listbox.curselection()
        if selected_indices:
            selected_node = self.node_listbox.get(selected_indices[0])
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert(tk.END, f"Details for Node: {selected_node}\n")
            # Add more details if needed (e.g., subscriptions, publications)
        else:
             self.details_text.delete("1.0", tk.END)


    def show_topic_details(self, event):
        selected_indices = self.topic_listbox.curselection()
        if selected_indices:
            selected_topic = self.topic_listbox.get(selected_indices[0])
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert(tk.END, f"Details for Topic: {selected_topic}\n")
            topics = self.get_topic_names_and_types()
            for topic_name, topic_type in topics:
                if topic_name == selected_topic:
                    self.details_text.insert(tk.END, f"  Type: {topic_type}\n")
                    break
            # Add more details if needed (e.g., publishers, subscribers)
        else:
            self.details_text.delete("1.0", tk.END)


    def show_service_details(self, event):
        selected_indices = self.service_listbox.curselection()
        if selected_indices:
            selected_service = self.service_listbox.get(selected_indices[0])
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert(tk.END, f"Details for Service: {selected_service}\n")
            services = self.get_service_names_and_types()
            for service_name, service_type in services:
                if service_name == selected_service:
                    self.details_text.insert(tk.END, f"  Type: {service_type}\n")
                    break
            # Add more details if needed (e.g., server, clients)
        else:
            self.details_text.delete("1.0", tk.END)


    def show_action_details(self, event):
        selected_indices = self.action_listbox.curselection()
        if selected_indices:
            selected_action = self.action_listbox.get(selected_indices[0])
            self.details_text.delete("1.0", tk.END)
            self.details_text.insert(tk.END, f"Details for Action: {selected_action}\n")
            # Add action details here (e.g., goal type, feedback type, result type) - Requires action introspection
        else:
            self.details_text.delete("1.0", tk.END)


    def run(self):
        # The Tkinter main loop runs in the main thread
        self.window.mainloop()


# Function to run the ROS2 spinning in a separate thread
def spin_ros_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    gui = TurtlesimGUI()

    # Create and start a separate thread for ROS2 spinning
    ros_thread = threading.Thread(target=spin_ros_node, args=(gui,))
    ros_thread.start()

    # Run the Tkinter GUI main loop in the main thread
    gui.run()

    # Join the ROS2 thread after the GUI window is closed
    ros_thread.join()


if __name__ == '__main__':
    main()
