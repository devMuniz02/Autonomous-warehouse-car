import os
os.environ["GST_DEBUG"] = "0"  # Suppress GStreamer debug output

import tkinter as tk
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import socket
import numpy as np
import queue
import math
import cv2  # OpenCV for image processing
from pyzbar.pyzbar import decode
import json  # For handling JSON coordinates
import statistics

IP_address = #INCLUDE HERE THE IP ADDRESS OF CAR
numberframes = 0
class RobotControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control GUI")
        self.running = True
        self.closed = False
        self.robot_started = False
        self.image_queue = queue.Queue()
        self.sensor_histories = [[], [], []]
        self.latest_lane_command = "Centered"
        self.numberframes = 0
        self.global_min = 0
        self.qr_read_delay = False  # Flag to delay QR reading
        self.command_delay = False  # Flag to delay command sending
        self.drop_off_button = None  # Placeholder for Drop Off button
        self.pick_up_button = None  # Placeholder for Pick Up button
        self.qr_action_pending = False  # Flag to indicate QR action is pending

        # --- Set up JSON for polygon coordinates ---
        json_file = 'coordinates.json'
        sample_data = {
            "coordinates": [
                {"x": 0.0, "y": 0.30},
                {"x": 0,   "y": 1},
                {"x": 1,   "y": 1},
                {"x": 1.0, "y": 0.30}
            ]
        }
        with open(json_file, "w") as file:
            json.dump(sample_data, file, indent=4)
        with open(json_file, "r") as file:
            data = json.load(file)
        if data is None:
            self.coords = [(0.25, 0.5), (0, 1), (1, 1), (0.75, 0.5)]
        else:
            coordinates = data["coordinates"]
            self.coords = [
                (coordinates[0]["x"], coordinates[0]["y"]),
                (coordinates[1]["x"], coordinates[1]["y"]),
                (coordinates[2]["x"], coordinates[2]["y"]),
                (coordinates[3]["x"], coordinates[3]["y"])
            ]

        # --- Configure grid weights for proper layout ---
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=2)
        self.root.grid_rowconfigure(2, weight=0)
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_columnconfigure(1, weight=1)

        # --- Layout Setup ---
        # Sensor Data Frame (top left)
        self.sensor_frame = tk.Frame(self.root, bd=2, relief="groove")
        self.sensor_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nw")
        self.sensor_labels = []
        self.sensor_histories = [[], [], []]  # Up to last 5 values for each sensor
        for i in range(3):
            label = tk.Label(self.sensor_frame, text=f"Sensor {i+1}: no data", font=("Helvetica", 12))
            label.pack(anchor="w", padx=5, pady=2)
            self.sensor_labels.append(label)
        
        # Radar (Rectangular) Plot Frame (bottom left)
        self.radar_frame = tk.Frame(self.root, bd=2, relief="groove")
        self.radar_frame.grid(row=1, column=0, padx=10, pady=(0,10), sticky="nsew")
        self.radar_frame.grid_rowconfigure(0, weight=1)
        self.radar_frame.grid_columnconfigure(0, weight=1)
        self.fig, self.ax = plt.subplots(figsize=(4, 3))
        self.ax.set_xlim(-0.5, 2.5)
        self.ax.set_ylim(0, 50)  # Maximum changed from 400 to 500
        self.ax.set_xlabel("Sensor lateral position (cm)")
        self.ax.set_ylabel("Distance (cm)")
        self.ax.set_title("Front Radar View")
        # Add danger, caution, and safe zones
        self.ax.axhspan(0, 10, color='red', alpha=0.5, label='Danger (0-20cm)')
        self.ax.axhspan(10, 40, color='yellow', alpha=0.5, label='Caution (20-40cm)')
        self.ax.axhspan(40, 50, color='green', alpha=0.5, label='Safe (40-50cm)')
        
        self.sensor_positions = [0, 1, 2]
        self.sensor_lines = []
        for x in self.sensor_positions:
            line, = self.ax.plot([x, x], [0, 0], lw=2, marker='o', markersize=5)
            self.sensor_lines.append(line)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.radar_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        
        
        # Camera Feed Frame (top right, spanning both rows)
        self.image_frame = tk.Frame(self.root, bd=2, relief="groove")
        self.image_frame.grid(row=0, column=1, rowspan=2, padx=10, pady=10, sticky="nsew")
        self.image_label = tk.Label(self.image_frame)
        self.image_label.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Second Image Frame (for red-detected area)
        self.red_image_frame = tk.Frame(self.root, bd=2, relief="groove")
        self.red_image_frame.grid(row=0, column=2, rowspan=2, padx=10, pady=10, sticky="nsew")
        self.red_image_label = tk.Label(self.red_image_frame)
        self.red_image_label.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Control Buttons and Status Message (bottom row)
        self.button_frame = tk.Frame(self.root)
        self.button_frame.grid(row=2, column=0, columnspan=4, pady=(0,10), sticky="ew")
        for i in range(7):
            self.button_frame.grid_columnconfigure(i, weight=1)
        self.start_button = tk.Button(self.button_frame, text="Start", command=self.start_robot, width=10)
        self.start_button.grid(row=0, column=0, padx=10, pady=5)
        self.drop_off_button = tk.Button(self.button_frame, text="Drop Off", command=self.handle_drop_off, width=10, state="disabled")
        self.drop_off_button.grid(row=0, column=1, padx=10, pady=5)
        self.pick_up_button = tk.Button(self.button_frame, text="Pick Up", command=self.handle_pick_up, width=10, state="disabled")
        self.pick_up_button.grid(row=0, column=2, padx=10, pady=5)
        self.stop_button = tk.Button(self.button_frame, text="Stop", command=self.stop_robot, width=10, state="disabled")
        self.stop_button.grid(row=0, column=3, padx=10, pady=5)
        self.close_button = tk.Button(self.button_frame, text="Close", command=self.on_closing, width=10)
        self.close_button.grid(row=0, column=4, padx=10, pady=5)
        self.status_label = tk.Label(self.button_frame, text="Status: Waiting for command", font=("Helvetica", 14))
        self.status_label.grid(row=0, column=5, padx=10, pady=5)
        self.label = tk.Label(self.button_frame, text=f"QR CODE: None detected", font=("Helvetica", 14))
        self.label.grid(row=0, column=6, padx=20, pady=5)
        self.label_zone = tk.Label(self.button_frame, text=f"Zone: Safe", font=("Helvetica", 14), fg = "green", bg = "black")
        self.label_zone.grid(row=1, column=6, padx=20, pady=5)
        self.setup_movement_buttons()

        # --- Initialize UDP Sockets for receiving data ---
        self.bufferSize = 40000
        msgFromClient = 'Hello World, from your client'
        bytesToSend = msgFromClient.encode('utf-8')
        
        self.serverAddress = (IP_address, 5555)   # For sensor (text) data
        self.imageAddress = (IP_address, 6666)      # For image data
        self.control_address = (IP_address, 3333)
        self.stop_address = (IP_address, 4444)
        
        self.text_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.stop_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print('Sending handshake to sensor server...', self.serverAddress)
        self.text_socket.sendto(bytesToSend, self.serverAddress)
        print('Sending handshake to image server...', self.imageAddress)
        self.image_socket.sendto(bytesToSend, self.imageAddress)
        print('Sending handshake to movement server...', self.control_socket)
        self.control_socket.sendto(bytesToSend, self.control_address)
        
        self.text_socket.setblocking(False)
        self.image_socket.settimeout(1.0)
        
        
        # Create queue and start thread for UDP image reception
        self.image_queue = queue.Queue()
        self.udp_thread = threading.Thread(target=self.receive_udp_images, daemon=True)
        self.udp_thread.start()
        
        # Schedule periodic updates
        self.image_update_id = self.root.after(30, self.update_udp_image)
        self.sensor_update_id = self.root.after(20, self.update_sensors)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def start_robot(self):
        self.robot_started = True
        
        self.running = True
        self.closed = False   
        self.numberframes = 0
        self.global_min = 0
        self.qr_read_delay = False  # Flag to delay QR reading
        self.command_delay = False  # Flag to delay command sending
        
        self.status_label.config(text="Status: Starting")
        self.label.config(text="QR CODE: None detected")
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.forward_button.config(state="disabled")
        self.left_button.config(state="disabled")
        self.stop_button2.config(state="disabled")
        self.right_button.config(state="disabled")
        self.backward_button.config(state="disabled")
        self.clockwise_button.config(state="disabled")
        self.counterclockwise_button.config(state="disabled")
        
        # Start sending control commands every 2 seconds
        try:
            self.control_socket.sendto("stop".encode('utf-8'), self.control_address)
            print("Sent control command: stop")
        except Exception as e:
            print("Error sending stop command:", e)
        self.send_control_command()
    
    def stop_robot(self):
        self.robot_started = False
        self.status_label.config(text="Status: Stopping")
        self.stop_button.config(state="disabled")
        self.start_button.config(state="normal")
        self.forward_button.config(state="normal")
        self.left_button.config(state="normal")
        self.stop_button2.config(state="normal")
        self.right_button.config(state="normal")
        self.backward_button.config(state="normal")
        self.clockwise_button.config(state="normal")
        self.counterclockwise_button.config(state="normal")
        
        # Send a stop command immediately
        try:
            self.control_socket.sendto("stop".encode('utf-8'), self.control_address)
            self.stop_socket.sendto("stop".encode('utf-8'), self.stop_address)
            print("Sent control command: stop")
        except Exception as e:
            print("Error sending stop command:", e)
    
    def send_control_command(self):
        """
        This function sends a control command to the robot only after receiving a message from the control socket.
        It runs every 100 ms instead of using a while loop.
        """ 
        if not self.robot_started:
            return
        if self.qr_action_pending:
            self.status_label.config(text=f"Loading zone, Status: Waiting... UR")
        if not self.command_delay:
            self.label.config(text=f"QR CODE: None detected")
            # Compute mean sensor values; if no data for a sensor, assume a high safe value.
            # Use latest lane detection result computed in update_udp_image.
            lane_result = getattr(self, "latest_lane_command", "Centered")
            command = 'stop'
        
            if self.global_min < 10:
                command = "stop"
                self.stop_socket.sendto("stop".encode('utf-8'), self.stop_address)
                self.control_socket.sendto("stop".encode('utf-8'), self.control_address)
                self.status_label.config(text=f"Status: {command}")
            elif 10 < self.global_min < 40:
                if lane_result == "Move Left":
                    command = "slow left"
                elif lane_result == "Move Right":
                    command = "slow right"
                else:
                    command = "slow forward"
                self.status_label.config(text=f"Status: {command}")
            else:
                if lane_result == "Move Left":
                    command = "left"
                elif lane_result == "Move Right":
                    command = "right"
                else:
                    command = "forward"
                self.label_zone.config(text=f"Zone: Safe")
                self.status_label.config(text=f"Status: {command}")

            print('Command:',command)
            try:
                self.control_socket.sendto(command.encode('utf-8'), self.control_address)
                print("Sent control command:", command)
            except Exception as e:
                print("Error sending control command:", e)

        # Schedule next execution in 100 ms
        self.root.after(3000, self.send_control_command)

    def update_udp_image(self):
        global numberframes
        """

        Retrieves a UDP image frame from the queue, applies a series of image transformations:
          - Gaussian blur and Canny edge detection.
          - Defines a polygon mask based on coordinates loaded from JSON.

          - Detects lines using the Hough transform.
          - Computes a rough lane center from the detected lines and compares it with the image center.
          - Overlays a message ("Move Left", "Move Right", or "Centered") on the final image.

        The final processed image is then updated in the Tkinter GUI.
        """
        try:
            while not self.image_queue.empty():
                frame = self.image_queue.get_nowait()
                if frame is None:
                    continue

                # Create a mask for black pixels in the color image
                # Define lower and upper bounds for black in HSV space or RGB
                # These values can be adjusted based on the image and lighting conditions.
                lower_black = np.array([0, 0, 0])  # Lower bound for black color (RGB)
                upper_black = np.array([80, 80, 80])  # Upper bound for black color (RGB)

                # Threshold the color image to extract black regions
                black_mask = cv2.inRange(frame, lower_black, upper_black)

                # Optionally, apply some morphological operations to clean up the mask
                kernel = np.ones((5, 5), np.uint8)
                black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)  # Close small gaps

                # Now apply Gaussian blur to the black mask to smooth edges
                blurred = cv2.GaussianBlur(black_mask, (3, 3), 0)

                # Apply edge detection on the black regions
                edges = cv2.Canny(blurred, 100, 255, apertureSize=3)
                height, width = edges.shape

                cropped_edges = edges

                # Detect lines with Hough transform
                lines = cv2.HoughLinesP(cropped_edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
                hough_image = frame.copy()  # Use the original color image for visualization
                angles = []

                if lines is not None:
                    for line in lines:
                        x_1, y_1, x_2, y_2 = line[0]

                        # Calculate the angle of the line using atan2
                        angle = math.degrees(math.atan2(y_2 - y_1, x_2 - x_1))

                        # Only consider vertical lines
                        if abs(angle) > 45:  # Only consider near-vertical lines
                            angles.append(angle)

                            # Draw the line on the image
                            cv2.line(hough_image, (x_1, y_1), (x_2, y_2), (0, 255, 0), 2)

                            # Overlay the angle on the image near the line
                            angle_text = f"{int(angle)}°"
                            angle_position = (int((x_1 + x_2) / 2), int((y_1 + y_2) / 2))
                            cv2.putText(hough_image, angle_text, angle_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Decide the movement based on detected angles
                if len(angles) == 0:
                    angles = [-90, 90]  # fallback if no lines detected

                min_angle = min(angles)
                max_angle = max(angles)

                # All angles are positive and less than 60 → move left
                if min_angle > 0 and max_angle < 50:
                    lane_command = "Move Left"

                # All angles are negative and greater than -60 → move right
                elif max_angle < 0 and min_angle > -50:
                    lane_command = "Move Right"

                # Mix of angles → check symmetry
                else:
                    total_angle = min_angle + max_angle

                    if -40 < total_angle < 40:
                        lane_command = "Centered"
                    elif total_angle >= 40:
                        lane_command = "Move Left"
                    else:
                        lane_command = "Move Right"

                        

                # Overlay the message on the final image
                cv2.putText(hough_image, lane_command, (50, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.putText(hough_image, f"Min-max angles: {min(angles),max(angles)}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                # Update the Tkinter label with the processed image
                hough_rgb = cv2.cvtColor(hough_image, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(hough_rgb).resize((320, 240))
                photo = ImageTk.PhotoImage(pil_img)
                self.image_label.config(image=photo)
                self.image_label.image = photo  # Keep a reference

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Step 1: Apply polygon mask
                height, width = frame.shape[:2]
                polygon = np.array([[[
                    (int(width * x), int(height * y)) for (x, y) in self.coords
                ]]], np.int32)

                # Create the mask and apply it
                mask = np.zeros((height, width), dtype=np.uint8)
                cv2.fillPoly(mask, polygon, 255)

                # Mask the original frame
                masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

                # Step 2: QR code detection in the masked area
                qr_data = None
                qr_codes = decode(masked_frame)
                for qr_code in qr_codes:
                    points = qr_code.polygon
                    if len(points) == 4:
                        pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                        cv2.polylines(frame, [pts], True, (0, 255, 0), 5)
                        
                    qr_data = qr_code.data.decode("utf-8")
                    print(f"QR Code Detected: {qr_data}")

                    rect_x, rect_y, rect_w, rect_h = qr_code.rect
                    cv2.putText(frame, qr_data, (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (60, 255, 255), 2, cv2.LINE_AA)
                    
                if not self.command_delay:
                    if qr_data in ['DROP-OFF', 'PICK-UP', 'HOME', 'TURN1', 'TURN2']:
                        self.label.config(text=f"QR CODE: {qr_data}")
                        self.status_label.config(text=f"Status: {qr_data}")
                        if self.robot_started:
                            self.command_delay = True
                            self.control_socket.sendto(qr_data.encode('utf-8'), self.control_address)
                        if qr_data in ['TURN1', 'TURN2']:
                            self.root.after(3000, self.reset_command_delay)  # 3-second delay
                        # Handle QR-specific actions
                        if qr_data == "DROP-OFF":
                            self.qr_action_pending = True
                            self.drop_off_button.config(state="normal")  # Enable Drop Off button
                        elif qr_data == "PICK-UP":
                            self.qr_action_pending = True
                            self.pick_up_button.config(state="normal")  # Enable Pick Up button
                        elif qr_data == "HOME":
                            self.stop_robot()  # Explicitly stop the robot
                            print("Robot sent home. Stopping all commands and robot.")
                    else:
                        self.latest_lane_command = lane_command
                
                # Step 4: Convert final frame for Tkinter display
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.polylines(frame_rgb, polygon, isClosed=True, color=(255, 0, 0), thickness=2)
                pil_img = Image.fromarray(frame_rgb).resize((320, 240))
                photo = ImageTk.PhotoImage(pil_img)

                # Step 5: Update the red image label with the processed frame
                self.red_image_label.config(image=photo)
                self.red_image_label.image = photo  # Keep a reference
        except Exception as e:
            print("Error in update_udp_image:", e)
        self.image_update_id = self.root.after(50, self.update_udp_image)
    
    def setup_movement_buttons(self):
        """
        Adds movement buttons for manual control of the robot.
        The available controls are:
          - Forward (↑)
          - Backward (↓)
          - Slide Left (←)
          - Slide Right (→)
          - Turn right (↻)
          - Turn left (↺)
        """
        self.movement_frame = tk.Frame(self.root)
        self.movement_frame.grid(row=3, column=0, columnspan=1, pady=5, sticky="ew")
        
        for i in range(3):
            self.movement_frame.grid_columnconfigure(i, weight=1)

        # Row 1 (Forward)
        self.forward_button = tk.Button(self.movement_frame, text="↑", font=("Helvetica", 14), command=lambda: self.send_manual_command("forward"), width=10)
        self.forward_button.grid(row=0, column=1, padx=5, pady=5)

        # Row 2 (Left, Stop, Right)
        self.left_button = tk.Button(self.movement_frame, text="←", font=("Helvetica", 14), command=lambda: self.send_manual_command("slide left"), width=10)
        self.left_button.grid(row=1, column=0, padx=5, pady=5)

        self.stop_button2 = tk.Button(self.movement_frame, text="■", font=("Helvetica", 14), command=lambda: self.send_manual_command("stop"), width=10)
        self.stop_button2.grid(row=1, column=1, padx=5, pady=5)

        self.right_button = tk.Button(self.movement_frame, text="→", font=("Helvetica", 14), command=lambda: self.send_manual_command("slide right"), width=10)
        self.right_button.grid(row=1, column=2, padx=5, pady=5)

        # Row 3 (Backward)
        self.backward_button = tk.Button(self.movement_frame, text="↓", font=("Helvetica", 14), command=lambda: self.send_manual_command("backward"), width=10)
        self.backward_button.grid(row=2, column=1, padx=5, pady=5)

        # Row 4 (Clockwise, Counterclockwise)
        self.clockwise_button = tk.Button(self.movement_frame, text="↻", font=("Helvetica", 14), command=lambda: self.send_manual_command("right"), width=10)
        self.clockwise_button.grid(row=0, column=2, padx=5, pady=5)

        self.counterclockwise_button = tk.Button(self.movement_frame, text="↺", font=("Helvetica", 14), command=lambda: self.send_manual_command("left"), width=10)
        self.counterclockwise_button.grid(row=0, column=0, padx=5, pady=5)
        
    def reset_qr_read_delay(self):
        """Reset the QR read delay flag."""
        self.qr_read_delay = False

    def reset_command_delay(self):
        """Reset the command delay flag."""
        self.command_delay = False
        
    def handle_drop_off(self):
        """Handle the Drop Off action."""
        self.qr_action_pending = False
        self.reset_command_delay()
        self.drop_off_button.config(state="disabled")  # Disable the button

    def handle_pick_up(self):
        """Handle the Pick Up action."""
        self.qr_action_pending = False
        self.reset_command_delay()
        self.pick_up_button.config(state="disabled")  # Disable the button

    def send_manual_command(self, command):
        """
        Sends a manual movement command to the robot.
        Commands include:
          - "forward"
          - "backward"
          - "slide left"
          - "slide right"
          - "right"
          - "left"
        """
        try:
            self.control_socket.sendto(command.encode('utf-8'), self.control_address)
            if command == "stop":
                self.stop_socket.sendto(command.encode('utf-8'), self.stop_address)
            self.status_label.config(text=f"Manual Control: {command}")
            
            print(f"Sent manual command: {command}")
        except Exception as e:
            print("Error sending manual control command:", e)

        
    def update_sensors(self):
        """
        Update sensor readings from the text socket. The sensor data is received as a vector
        that can have 1 to 3 distances (e.g. "[5]", "[5,10]", or "[5,10,15]"). For each sensor,
        if data is received, update its history (keeping up to 5 values) and display the mean.
        If no data has ever been received for a sensor, display "no data".
        """
        while not self.closed:
            try:
                text_data, _ = self.text_socket.recvfrom(self.bufferSize)
                message = text_data.decode('utf-8').strip().strip("[]")
                if message:
                    parts = message.split(',')
                    values = []
                    for part in parts:
                        try:
                            values.append(float(part.strip()))
                        except Exception as e:
                            print("Error converting sensor value:", e)
                    for i in range(len(values)):
                        self.sensor_histories[i].append(values[i])
                        if len(self.sensor_histories[i]) > 5:
                            self.sensor_histories[i].pop(0)
                else:
                    print("Received empty sensor message.")
            except Exception:
                break

        for i in range(3):
            if len(self.sensor_histories[i]) == 0:
                display_text = "no data"
                median_value = 0
            else:
                median_value = statistics.median(self.sensor_histories[i])
                display_text = f"{median_value:.2f} cm"
            self.sensor_labels[i].config(text=f"Sensor {i+1}: {display_text}")
            self.sensor_lines[i].set_data([self.sensor_positions[i], self.sensor_positions[i]], [0, median_value])
        self.canvas.draw()
        
        sensor_medians = []
        for hist in self.sensor_histories:
            if len(hist) > 0:
                sensor_medians.append(statistics.median(hist))
            else:
                sensor_medians.append(10)  # Default low value if no data

        self.global_min = min(sensor_medians)
        if self.global_min < 10:
            self.label_zone.config(text=f"Zone: Danger", fg = "red", bg = "black")
        elif 10 < self.global_min < 40:
            self.label_zone.config(text=f"Zone: Steering", fg = "yellow", bg = "black")
        else:
            self.label_zone.config(text=f"Zone: Safe", fg = "green", bg = "black")
            
        self.sensor_update_id = self.root.after(20, self.update_sensors)
    
    def receive_udp_images(self):
        """
        Runs in a separate thread to receive UDP image packets.
        Decodes the image using OpenCV and puts it into a thread-safe queue.
        """
        while not self.closed:
            try:
                packet, addr = self.image_socket.recvfrom(self.bufferSize)
                np_arr = np.frombuffer(packet, dtype=np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if img is not None:
                    self.image_queue.put(img)
                else:
                    print("Warning: Received packet could not be decoded as an image.")
            except socket.timeout:
                continue
            except Exception as e:
                print("Error receiving UDP image:", e)
                continue
    
    def on_closing(self):
        if self.closed:
            return
        self.closed = True
        self.running = False
        self.root.after_cancel(self.image_update_id)
        self.root.after_cancel(self.sensor_update_id)
        self.text_socket.close()
        self.image_socket.close()
        self.control_socket.close()
        self.stop_socket.close()
        self.root.destroy()
        print("Closing GUI...")
        app.on_closing()

def terminal_exit_listener(app):
    """
    Listens on the terminal for the 'exit' or 'quit' command.
    When received, closes the GUI.
    """
    while True:
        command = input("Type 'exit' or 'quit' to close the GUI: ")
        if command.strip().lower() in ("exit", "quit"):
            print("Closing GUI...")
            app.on_closing()
            break

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlGUI(root)
    threading.Thread(target=terminal_exit_listener, args=(app,), daemon=True).start()
    root.mainloop()

