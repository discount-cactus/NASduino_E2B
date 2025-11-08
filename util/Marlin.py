import tkinter as tk
import usb.core
import usb.util
import serial
import serial.tools.list_ports
import threading
import time
from PIL import ImageGrab
import os
from tkinter import PhotoImage

version_number = "1.0"

# Define window dimensions and background color
window_width = 800
window_height = 800
window_background_col = '#080840'
button_col = '#4287f5'
button_col2 = '#66A0FF'
button_text_col = 'black'
button_width = 15
button_height = 2

center_x = window_width // 2
center_y = window_height // 2

SELECTED_DEVICE = None  # This will store the serial port of the selected device
SELECTED_DEVICE_IDENTIFIER = None  # This will store the full device identifier (e.g., "0x2341:0x0043 (Arduino)")
SELECTED_DEVICE_ARCH = None  # To store the architecture (Arduino/ESP32)
SELECTED_DEVICE_DESCRIPTION = None  # Store the device description
SELECTED_DEVICE_SERIALNUMBER = None  # Store the device serial number
SELECTED_DEVICE_MASTERCOUNT = None
SELECTED_DEVICE_MAXDEVICES = None
SELECTED_DEVICE_SSDCOUNT = None

averageArrivalRate = 0.0
averageReadSpeed = 0
averageWriteSpeed = 0
averageRoundTripPacketSpeed = 0
bitErrorRate = 0

# Class to represent a green box
class GreenBox:
    def __init__(self, canvas, x, y, width, height, color="green"):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        
        # Create the green box on the canvas
        self.box = self.canvas.create_rectangle(self.x, self.y, self.x + self.width, self.y + self.height, fill=self.color)

    def update(self, new_x, new_y):
        # Update the position of the box
        self.canvas.coords(self.box, new_x, new_y, new_x + self.width, new_y + self.height)
        self.x = new_x
        self.y = new_y


# Class to represent a gray box
class GrayBox:
    def __init__(self, canvas, x, y, width, height, color="gray"):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        
        # Create the gray box on the canvas
        self.box = self.canvas.create_rectangle(self.x, self.y, self.x + self.width, self.y + self.height, fill=self.color)

    def update(self, new_x, new_y):
        # Update the position of the box
        self.canvas.coords(self.box, new_x, new_y, new_x + self.width, new_y + self.height)
        self.x = new_x
        self.y = new_y

# Create the root window
root = tk.Tk()
root.title("Marlin UI")
root.geometry(f"{window_width}x{window_height}")
root.configure(bg=window_background_col)

# Define Arduino and ESP32 Vendor and Product IDs
arduino_vendor_id = 0x2341
esp32_vendor_id = 0x303A

# Function to scan and detect USB devices, filtering for Arduino or ESP32
def scan_usb_devices():
    devices = []
    # Get all available USB serial ports
    ports = serial.tools.list_ports.comports()

    for port in ports:
        try:
            # Add the port name and description to the list of devices
            devices.append({
                "device_id": port.device,  # Port name (e.g., /dev/cu.usbmodem1412101)
                "description": port.description  # Description (e.g., device description)
            })
        except Exception as e:
            print(f"Error checking port {port.device}: {e}")
    
    return devices

# Function to update the dropdown menu with active USB serial ports
def connect_to_device():
    global SELECTED_DEVICE

    connect_button.config(text="Select USB Device")

    # Scan for all active USB serial ports
    devices = scan_usb_devices()

    if devices:
        # Create a dropdown menu to select a USB device
        device_var.set("None")  # Set "None" as the default selection
        device_menu['menu'].delete(0, 'end')  # Clear the current menu options
        device_menu['menu'].add_command(label="None", command=lambda: on_device_select("None"))  # Add the "None" option
        for device in devices:
            # Add each device's name and description to the dropdown
            device_menu['menu'].add_command(label=f"{device['device_id']} ({device['description']})", 
                                            command=lambda device_id=device['device_id']: on_device_select(device_id))
        
        # Show the dropdown menu after the button is clicked, positioned below the button
        device_menu.place(x=center_x - (button_width * 6), y=55)
        device_menu.tkraise()
        
        # Bind the click event to detect clicks outside the dropdown menu
        root.bind("<Button-1>", on_outside_click)  # Detect clicks outside the menu
    else:
        print("No USB devices found!")
        connect_button.config(text="Connect")


# Function to handle clicks outside the dropdown menu
def on_outside_click(event):
    global SELECTED_DEVICE, SELECTED_DEVICE_IDENTIFIER

    if device_menu.winfo_ismapped():  # Check if the dropdown is currently visible
        x1, y1, x2, y2 = device_menu.bbox("menu")  # Get the coordinates of the dropdown menu
        
        # Check if the click is outside the bounding box of the dropdown menu
        if not (x1 <= event.x <= x2 and y1 <= event.y <= y2):
            # Hide the dropdown menu
            device_menu.place_forget()
            connect_button.config(text="Connect")  # Reset the button text
            root.unbind("<Button-1>")  # Unbind the click event to prevent multiple triggers

# Function to get the device architecture and identifier
def get_device_information():
    # Local variables to store the device information
    device_arch = "Unknown"
    device_identifier = "Unknown Device"
    device_description = "Unknown"
    device_serial_number = "Unknown"

    # Check if SELECTED_DEVICE is None or not valid
    if SELECTED_DEVICE is None:
        return device_arch, device_identifier, device_description, device_serial_number  # Return default values

    # Scan all available USB serial ports
    ports = serial.tools.list_ports.comports()

    # Iterate through all the available ports
    for port in ports:
        try:
            # Check if the port matches the selected device
            if port.device == SELECTED_DEVICE:
                # If manufacturer contains "Espressif", it's an ESP32 device
                if "Espressif" in port.manufacturer:
                    device_arch = "ESP32"
                # If manufacturer contains "Arduino", it's an Arduino device
                elif "Arduino" in port.manufacturer:
                    device_arch = "Arduino"
                else:
                    device_arch = "Unknown"

                # Set the selected device identifier with the full details
                device_identifier = f"{port.device} ({port.description})"
                
                # Store the device description and serial number
                device_description = port.description
                device_serial_number = port.serial_number

                # Break after processing the selected device
                break

        except Exception as e:
            print(f"Error checking port {port.device}: {e}")

    return device_arch, device_identifier, device_description, device_serial_number

# Function to handle selection from the dropdown menu
def on_device_select(device_id):
    global SELECTED_DEVICE, SELECTED_DEVICE_IDENTIFIER, SELECTED_DEVICE_ARCH
    global SELECTED_DEVICE_DESCRIPTION, SELECTED_DEVICE_SERIALNUMBER

    # If "None" is selected, set all variables to "Unknown"
    if device_id == "None":
        SELECTED_DEVICE = None
        SELECTED_DEVICE_ARCH = "Unknown"
        SELECTED_DEVICE_IDENTIFIER = "Unknown"
        SELECTED_DEVICE_DESCRIPTION = "Unknown"
        SELECTED_DEVICE_SERIALNUMBER = "Unknown"

        # Update the labels to reflect the "Unknown" state
        label_device_arch.config(text=f"Architecture: {SELECTED_DEVICE_ARCH}")
        label_device_id.config(text=f"ID: {SELECTED_DEVICE_IDENTIFIER}")
        label_device_description.config(text=f"Description: {SELECTED_DEVICE_DESCRIPTION}")
        label_device_serialnumber.config(text=f"Serial Number: {SELECTED_DEVICE_SERIALNUMBER}")
        selected_device_label.config(text="Device: None")

        # Hide the dropdown menu and reset the button text
        device_menu.place_forget()
        connect_button.config(text="Connect")
        root.unbind("<Button-1>")  # Unbind the click event when the selection is made
        return  # Exit the function as no further action is needed

    # Save the selected device's serial port
    SELECTED_DEVICE = device_id  # Update to use "device_id" instead of "serial_port"

    # Call get_device_information() to get the device information
    SELECTED_DEVICE_ARCH, SELECTED_DEVICE_IDENTIFIER, SELECTED_DEVICE_DESCRIPTION, SELECTED_DEVICE_SERIALNUMBER = get_device_information()

    # Update the labels with the obtained values
    label_device_arch.config(text=f"Architecture: {SELECTED_DEVICE_ARCH}")
    label_device_id.config(text=f"ID: {SELECTED_DEVICE_IDENTIFIER}")
    label_device_description.config(text=f"Description: {SELECTED_DEVICE_DESCRIPTION}")
    label_device_serialnumber.config(text=f"Serial Number: {SELECTED_DEVICE_SERIALNUMBER}")
    
    # Update the selected device label to show the selected device's address
    selected_device_label.config(text=f"Device: {SELECTED_DEVICE}")
    print(f"Selected device: {SELECTED_DEVICE}")

    # Hide the dropdown menu
    device_menu.place_forget()

    # Reset the button text to "Connect"
    connect_button.config(text="Connect")
    root.unbind("<Button-1>")  # Unbind the click event when the selection is made

# Polling function to detect device disconnection
def monitor_device():
    global SELECTED_DEVICE
    while True:
        if SELECTED_DEVICE is not None:
            # Check if the selected device is still connected
            devices = scan_usb_devices()
            device_found = False

            for device in devices:
                if device["device_id"] == SELECTED_DEVICE:
                    device_found = True
                    break

            if not device_found:
                print("Device disconnected")
                # Create a small popup window (Toplevel) when the device is disconnected
                popup = tk.Toplevel()
                popup.geometry("250x150")
                popup.configure(bg='red')
                #popup.title("Device Disconnected")
                label = tk.Label(popup, text="Device disconnected!", width=250, height=150, bg='red', font=("Arial", 18))
                label.pack(expand=True)

                 # Ensure the popup is always on top of the root window
                popup.lift()  # Bring the popup window to the top
                popup.attributes("-topmost", True)  # Keep the popup on top
                
                # Close the popup when the window is closed
                popup.protocol("WM_DELETE_WINDOW", lambda: on_device_disconnected(popup))

                # Reset global variables when the device is disconnected
                SELECTED_DEVICE = None

        time.sleep(2)  # Poll every 2 seconds

def on_device_disconnected(popup):
    global SELECTED_DEVICE
    
    # Close the popup window
    popup.destroy()
    
    # Reset device information and UI elements
    SELECTED_DEVICE_ARCH = "Unknown"
    SELECTED_DEVICE_IDENTIFIER = "Unknown"
    SELECTED_DEVICE_DESCRIPTION = "Unknown"
    SELECTED_DEVICE_SERIALNUMBER = "Unknown"
    SELECTED_DEVICE_MASTERCOUNT = "Unknown"

    SELECTED_DEVICE_SSDCOUNT = "Unknown"
    SELECTED_DEVICE_MAXDEVICES = "Unknown"
    
    # Update the labels to reflect the disconnection
    label_device_arch.config(text=f"Architecture: {SELECTED_DEVICE_ARCH}")
    label_device_id.config(text=f"ID: {SELECTED_DEVICE_IDENTIFIER}")
    label_device_description.config(text=f"Description: {SELECTED_DEVICE_DESCRIPTION}")
    label_device_serialnumber.config(text=f"Serial Number: {SELECTED_DEVICE_SERIALNUMBER}")
    label_device_master_count.config(fg='black', text=f"Architecture: {SELECTED_DEVICE_MASTERCOUNT}")

    label_device_ssd_count.config(text=f"SSD Count: {SELECTED_DEVICE_SSDCOUNT}")
    label_max_devices.config(fg='black', text=f"ID: {SELECTED_DEVICE_MAXDEVICES}")
    selected_device_label.config(text=f"Device: None")
    
    # You can add additional logic that you want to run when the device is disconnected
    # For example, you can call other functions or reset variables.

def run_diagnostics():
    import ast
    global SELECTED_DEVICE_SSDCOUNT

    ssd_blocks = []

    #button_diagnostics.config(state=tk.DISABLED)
    
    timeoutVal = 5
    try:
        with serial.Serial(SELECTED_DEVICE, 115200, timeout=timeoutVal) as ser:
            time.sleep(1)
            ser.write(b'A')  # Send command followed by newline
            #print("Sent A command to Arduino")

            start_time = time.time()
            response = ''
            
            while True:
                # Check if there's data available
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    print(f"Received from Arduino: {response}")

                    # Convert the CSV string to an array of integers
                    #data_array = [x for x in response.split(',')]
                    response_wrapped = f"[{response}]"
                    data_array = ast.literal_eval(response_wrapped)
                    print(f"Converted data to array: {data_array}")
                    array_indexes = len(data_array)
                    print(f"Number of indexes in data_array: {array_indexes}")
                    
                    # Assuming the data array has values for various fields
                    SELECTED_DEVICE_SSDCOUNT = int(data_array[0])
                    SELECTED_DEVICE_MASTERCOUNT = int(data_array[1])
                    SELECTED_DEVICE_MAXDEVICES = int(data_array[2])
                    averageArrivalRate = float(data_array[3])
                    if array_indexes > 4:
                        # Iterate over the remaining elements (blocks)
                        for i in range(4, array_indexes):
                            block = data_array[i]  # This is already a list
                            # Make sure all elements are ints
                            values = [int(x) for x in block]
                            ssd_blocks.append(values)
                            print(f"SSD block {i-3}: {values}")

                    print("All SSD blocks:", ssd_blocks)
                    
                    # Update the labels with the new values
                    label_device_ssd_count.config(text=f"SSD Count: {SELECTED_DEVICE_SSDCOUNT}")
                    label_device_master_count.config(text=f"Master Nodes: {SELECTED_DEVICE_MASTERCOUNT}")
                    label_max_devices.config(text=f"Max Devices: {SELECTED_DEVICE_MAXDEVICES}")
                    label_avg_arrival_rate.config(text=f"Avg Arrival Rate: {averageArrivalRate}")

                    # Create the display for SSD boxes after receiving response
                    create_ssd_display(SELECTED_DEVICE_SSDCOUNT)
                    break
                
                # Check if we've waited long enough
                if time.time() - start_time > timeoutVal:
                    print("Timeout: No response received from Arduino")
                    #button_diagnostics.config(state=tk.NORMAL)
                    break
                
                time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Error with serial communication: {e}")
        #button_diagnostics.config(state=tk.NORMAL)

#Displays the connected devices in frame_display
def create_ssd_display(ssd_count):
    # Clear the canvas before adding new boxes
    for widget in frame_display.winfo_children():
        widget.destroy()

    # Create the canvas to hold the boxes (inside the frame_display Frame)
    canvas = tk.Canvas(frame_display, width=frame_display_width, height=300, bg=button_col, bd=0, relief="flat", highlightthickness=0)
    canvas.pack(fill="both", expand=True)

    # Calculate the dynamic x-positions based on the canvas width
    gray_box_x = frame_display_width / 3  # 1/3 of the canvas width
    green_box_x = frame_display_width * 2 / 3  # 2/3 of the canvas width

    # Create the gray box in the middle using GrayBox class
    gray_box_y = 100
    gray_box_width = 100
    gray_box_height = 100
    gray_box = GrayBox(canvas, gray_box_x, gray_box_y, gray_box_width, gray_box_height)

    # Calculate the middle-right edge of the gray box
    gray_box_middle_right_x = gray_box_x + gray_box_width
    gray_box_middle_y = gray_box_y + gray_box_height / 2

    # Create the green boxes based on SSD count using GreenBox class
    green_box_width = 50
    green_box_height = 30
    vertical_spacing = 300 / ssd_count if ssd_count > 0 else 1  # To space them equally in the given height

    green_boxes = []
    for i in range(ssd_count):
        y_position = 20 + i * vertical_spacing  # Top padding + equal spacing between boxes
        green_box = GreenBox(canvas, green_box_x, y_position, green_box_width, green_box_height)

        # Calculate the middle-left edge of each green box
        green_box_middle_left_x = green_box_x
        green_box_middle_y = y_position + green_box_height / 2

        # Draw a line from the gray box to each green box
        canvas.create_line(gray_box_middle_right_x, gray_box_middle_y, green_box_middle_left_x, green_box_middle_y)

        green_boxes.append(green_box)


# Runs speed test for the connected device
def run_speedtest():
    global SELECTED_DEVICE, averageReadSpeed, averageWriteSpeed, averageRoundTripPacketSpeed, bitErrorRate

    #button_speedtest.config(state=tk.DISABLED)

    timeoutVal = 60
    try:
        with serial.Serial(SELECTED_DEVICE, 115200, timeout=timeoutVal) as ser:
            time.sleep(1)
            ser.write(b'B')  # Send command followed by newline
            #print("Sent B command to Arduino")

            start_time = time.time()
            response = ''
            
            while True:
                # Check if there's data available
                if ser.in_waiting > 0:
                    response = ser.readline().decode('utf-8').strip()
                    print(f"Received from Arduino: {response}")
                    
                    # --- Parse the response ---
                    parts = [p.strip() for p in response.split(',')]
                    if len(parts) == 4:
                        averageReadSpeed = float(parts[0])
                        averageWriteSpeed = float(parts[1])
                        averageRoundTripPacketSpeed = float(parts[2])
                        bitErrorRate = float(parts[3])

                        print(f"Read Speed: {averageReadSpeed} ms")
                        print(f"Write Speed: {averageWriteSpeed} ms")
                        print(f"Round Trip: {averageRoundTripPacketSpeed} ms")
                        print(f"BER: {bitErrorRate} %")
                        label_avg_read_speed.config(text=f"Avg Read Speed: {averageReadSpeed:.2f} ms")
                        label_avg_write_speed.config(text=f"Avg Write Speed: {averageWriteSpeed:.2f} ms")
                        label_avg_RTT_speed.config(text=f"Avg Round-Trip Time (RTT): {averageRoundTripPacketSpeed:.2f} ms")
                        label_avg_BER.config(text=f"BER: {bitErrorRate:.4f}")
                    else:
                        print("Unexpected response format:", response)
                    #button_speedtest.config(state=tk.NORMAL)
                    break
                
                # Check if we've waited long enough
                if time.time() - start_time > timeoutVal:
                    print("Timeout: No response received from Arduino")
                    #button_speedtest.config(state=tk.NORMAL)
                    break
                
                time.sleep(0.1)
    except serial.SerialException as e:
        print(f"Error with serial communication: {e}")
        #button_speedtest.config(state=tk.NORMAL)

#Generates a window to display the network model of the NAS we are interfacing with
def generate_network_model():
    global SELECTED_DEVICE_SSDCOUNT

    #button_netowrk_model.config(state=tk.DISABLED)

    # Create a new window for the network model visualization
    top = tk.Toplevel(root)
    top.title("NAS Network Model")
    
    # Window Size
    window_width = 600
    window_height = 600
    top.geometry(f"{window_width}x{window_height}")
    
    # Parameters
    num_masters = 1  # Number of master devices
    num_slaves = SELECTED_DEVICE_SSDCOUNT  # Number of slave devices (SSDs connected to the controller)
    
    # Arrival rate (λ): Number of requests per second
    T = 2  # Time between requests (seconds)
    lambda_ = num_masters * (1 / T)  # Total arrival rate (requests per second)
    
    # Service rate (μ): Controller processes 1 request every 1 second
    S = 1  # Time to process each request (seconds)
    mu_ = 1 / S  # Service rate (requests per second)
    
    # Queue length (k): Number of slave devices
    k = num_slaves  # The number of devices waiting for service
    
    # Max queue length (k_max): Assuming the system can handle all devices in the queue at once
    k_max = 12  # Maximum queue length can be defined by system capacity
    
    # Canvas to draw the model
    canvas = tk.Canvas(top, width=window_width, height=window_height, bg="white")
    canvas.pack(fill="both", expand=True)
    
    # Draw the controller (gray box in the center)
    gray_box_width = 100
    gray_box_height = 100
    gray_box_x = window_width // 2 - gray_box_width // 2
    gray_box_y = window_height // 2 - gray_box_height // 2
    gray_box = canvas.create_rectangle(gray_box_x, gray_box_y, 
                                       gray_box_x + gray_box_width, gray_box_y + gray_box_height,
                                       fill="gray", outline="black", width=2)
    
    # Draw master devices (represented as small circles aligned along the y-axis)
    master_radius = 20
    master_spacing = (window_height) / (num_masters + 1)  # Evenly distribute along y-axis
    master_devices = []
    
    # Move the blue circles 50 pixels to the left
    for i in range(num_masters):
        x_position = window_width // 4 - 50  # Align master devices to 1/4 of the window width, move left by 50
        y_position = master_spacing * (i + 1)  # Evenly spaced on the y-axis
        # Draw lines first, then circles
        canvas.create_line(x_position + master_radius, y_position, gray_box_x, gray_box_y + gray_box_height // 2,
                           arrow=tk.LAST, width=2)
        master_device = canvas.create_oval(x_position - master_radius, y_position - master_radius,
                                           x_position + master_radius, y_position + master_radius,
                                           fill="blue", outline="black", width=2)
        master_devices.append(master_device)
    
    # Draw slave devices (represented as green boxes aligned along the x-axis)
    green_box_width = 80
    green_box_height = 40
    slave_spacing = (window_height) / (num_slaves + 1)  # Evenly distribute along y-axis
    slave_devices = []
    
    # Move the green boxes 50 pixels to the right
    for i in range(num_slaves):
        x_position = window_width * 3 // 4 + 50  # Align slave devices to 3/4 of the window width, move right by 50
        y_position = slave_spacing * (i + 1)  # Evenly spaced on the y-axis
        # Draw lines first, then the green boxes
        canvas.create_line(gray_box_x + gray_box_width, gray_box_y + gray_box_height // 2,
                           x_position - green_box_width // 2, y_position,
                           arrow=tk.LAST, width=2)
        slave_device = canvas.create_rectangle(x_position - green_box_width // 2, y_position - green_box_height // 2,
                                               x_position + green_box_width // 2, y_position + green_box_height // 2,
                                               fill="green", outline="black", width=2)
        slave_devices.append(slave_device)
    
    # Display the queue (optional visualization: we can show a simple queue bar)
    queue_bar_x = gray_box_x
    queue_bar_y = gray_box_y + 200
    queue_bar_height = 30
    queue_bar_width = 130  # Each waiting device takes 30 pixels of space
    canvas.create_rectangle(queue_bar_x, queue_bar_y, queue_bar_x + queue_bar_width, 
                            queue_bar_y + queue_bar_height, fill="yellow", outline="black", width=2)
    
    # Add labels for visual clarity
    canvas.create_text(gray_box_x + gray_box_width // 2, gray_box_y + gray_box_height // 2,
                       text="Controller", fill="black", font=('Arial', 12, 'bold'))
    
    for i, master_device in enumerate(master_devices):
        x_position = window_width // 4 - 50  # Align master devices to 1/4 of the window width, move left by 50
        canvas.create_text(x_position, master_spacing * (i + 1) - master_radius - 10, 
                           text=f"Master {i+1}", fill="black", font=('Arial', 14))
    
    for i, slave_device in enumerate(slave_devices):
        y_position = slave_spacing * (i + 1)
        canvas.create_text(window_width * 3 // 4 + 50, y_position, text=f"SSD {i+1}", fill="black", font=('Arial', 14))

    canvas.create_text(queue_bar_x + queue_bar_width // 2, queue_bar_y + queue_bar_height // 2, 
                       text=f"Queue Length: {k}", fill="black", font=('Arial', 14))
    
    # Display all the calculated values as labels (positioned under the queue box)
    label_x = gray_box_x + gray_box_width // 2  # Center text horizontally with the controller
    label_y = queue_bar_y + queue_bar_height + 10  # Set the Y position just below the queue bar
    canvas.create_text(label_x, label_y, text=f"λ (Arrival Rate): {lambda_} requests/sec", fill="black", font=('Arial', 14))
    canvas.create_text(label_x, label_y + 20, text=f"μ (Service Rate): {mu_} requests/sec", fill="black", font=('Arial', 14))
    canvas.create_text(label_x, label_y + 40, text=f"k (Queue Length): {k}", fill="black", font=('Arial', 14))
    canvas.create_text(label_x, label_y + 60, text=f"k_max (Max Queue Capacity): {k_max}", fill="black", font=('Arial', 14))
    print("Network Model Parameters:")
    print(f"λ (Arrival Rate): {lambda_} requests/sec")
    print(f"μ (Service Rate): {mu_} requests/sec")
    print(f"k (Queue Length): {k}")
    print(f"k_max (Max Queue Capacity): {k_max}")

    #button_netowrk_model.config(state=tk.NORMAL)


marlin_icon = PhotoImage(file="marlin_icon.png")
icon_label = tk.Label(root, image=marlin_icon, bg=window_background_col)
icon_label.place(x=center_x - 390, y=10)
#icon_label.pack(side="left", padx=(0, 5))

#label_text = f"Marlin UI v{version_number}"
label_version_number = tk.Label(root, text=f"Marlin UI v{version_number}", bg=window_background_col, fg="white", font=("Arial", 24))
label_version_number.place(x=center_x - 320, y=15)

# Create a "Connect" button and place it at the top center
connect_button = tk.Button(root, text="Connect", width=button_width, height=button_height, bg=button_col, fg=button_text_col, font=("Arial", 16), command=connect_to_device)
connect_button.place(x=center_x - (button_width * 6), y=10)

# Create a label for the device and place it to the right of the "Connect" button
# Create a Frame to display the device address inside a box
device_label_frame = tk.Frame(root, bg="white", width=350, height=44, bd=2, relief="solid")
device_label_frame.place(x=center_x - 10 + (button_width * 4), y=8)
selected_device_label = tk.Label(device_label_frame, text="Device: None", bg="white", fg="black", font=("Arial", 18))
selected_device_label.place(relx=0.5, rely=0.5, anchor="center")

# Create a tkinter variable for storing the selected USB device
device_var = tk.StringVar()

# Create an empty dropdown menu (OptionMenu), initially hidden
device_menu = tk.OptionMenu(root, device_var, "")

# Create frame_display and frame_data
frame_display_width = window_width * 0.9  # 90% of the window width
frame_display = tk.Frame(root, bg=button_col, width=frame_display_width, height=300, bd=0, relief="flat", padx=0, pady=0)
frame_display.place(x=center_x - (frame_display_width // 2), y=80)


frame_data = tk.Frame(root, bg=button_col, width=frame_display_width, height=370)
frame_data.place(x=center_x - (frame_display_width // 2), y=400)  # Placed below frame_display

frame_data_buttons = tk.Frame(frame_data, bg=button_col2, width=frame_display_width, height=40)  # Red background
frame_data_buttons.pack(side="top", fill="x")

frame_data_controller = tk.Frame(frame_data, bg=button_col, width=frame_display_width / 2, height=330)
frame_data_controller.pack(side="left", fill="both", expand=True)

frame_data_times = tk.Frame(frame_data, bg=button_col, width=frame_display_width / 2, height=330)  # Yellow background
frame_data_times.pack(side="right", fill="both", expand=True)

# Number of buttons and their positions
num_buttons = 3
button_spacing = (frame_display_width - (button_width * num_buttons)) / (num_buttons + 1)

button_diagnostics = tk.Button(frame_data_buttons, text="Run Diagnostics", width=button_width, height=button_height, bg=button_col, fg=button_text_col, font=("Arial", 14), command=run_diagnostics)
button_diagnostics.place(x=button_spacing, rely=0.05)

button_speedtest = tk.Button(frame_data_buttons, text="Run Speed Test", width=button_width, height=button_height, bg=button_col, fg=button_text_col, font=("Arial", 14), command=run_speedtest)
button_speedtest.place(x=button_spacing * 2 + button_width, rely=0.05)

button_network_model = tk.Button(frame_data_buttons, text="Generate Model", width=button_width, height=button_height, bg=button_col, fg=button_text_col, font=("Arial", 14), command=generate_network_model)
button_network_model.place(x=button_spacing * 3 + button_width * 2, rely=0.05)


label_frame_data_controller = tk.Label(frame_data_controller, text="Controller Data", bg=button_col, fg="black", font=("Arial", 16, "bold"))
label_frame_data_controller.place(relx=0.5, rely=0.05, anchor="center")

label_device_arch = tk.Label(frame_data_controller, text="Architecture: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_arch.place(relx=0.02, rely=0.1)

label_device_id = tk.Label(frame_data_controller, text="ID: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_id.place(relx=0.02, rely=0.2)

label_device_description = tk.Label(frame_data_controller, text="Description: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_description.place(relx=0.02, rely=0.3)

label_device_serialnumber = tk.Label(frame_data_controller, text="Serial Number: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_serialnumber.place(relx=0.02, rely=0.4)

label_device_ssd_count = tk.Label(frame_data_controller, text="SSD Count: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_ssd_count.place(relx=0.02, rely=0.5)

label_device_master_count = tk.Label(frame_data_controller, text="Master Nodes: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_device_master_count.place(relx=0.02, rely=0.6)

label_max_devices = tk.Label(frame_data_controller, text="Max Devices: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_max_devices.place(relx=0.02, rely=0.7)



label_frame_data_times = tk.Label(frame_data_times, text="Performance Metrics Area", bg=button_col, fg="black", font=("Arial", 16, "bold"))
label_frame_data_times.place(relx=0.5, rely=0.05, anchor="center")

label_avg_read_speed = tk.Label(frame_data_times, text="Avg Read Speed: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_avg_read_speed.place(relx=0.02, rely=0.1)

label_avg_write_speed = tk.Label(frame_data_times, text="Avg Write Speed: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_avg_write_speed.place(relx=0.02, rely=0.2)

label_avg_RTT_speed = tk.Label(frame_data_times, text="Avg Rount-Trip Time (RTT): N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_avg_RTT_speed.place(relx=0.02, rely=0.3)

label_avg_BER = tk.Label(frame_data_times, text="BER: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_avg_BER.place(relx=0.02, rely=0.4)

label_avg_arrival_rate = tk.Label(frame_data_times, text="Avg Arrival Rate: N/A", bg=button_col, fg="black", font=("Arial", 14), anchor="w")
label_avg_arrival_rate.place(relx=0.02, rely=0.5)

# Start the polling in a separate thread
monitor_thread = threading.Thread(target=monitor_device, daemon=True)
monitor_thread.start()

# Run the application
root.mainloop()
