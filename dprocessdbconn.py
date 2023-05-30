import serial
import pandas as pd
import time
from datetime import datetime, timedelta
from collections import deque
from sqlalchemy import create_engine


# Serial port configuration
port = "/dev/ttyACM0"  
baudrate = 115200

# Battery parameters
nominal_voltage = 48.1
maximum_voltage = 54.0
minimum_voltage = 43.2
capacity = 52.0
nominal_energy = 2.5

# Moving average window size (0.25 seconds at 1kHz frequency)
window_size = 250

# Initialize moving average queues
vbatt_queue = deque(maxlen=window_size)
ibatt_queue = deque(maxlen=window_size)
vmot_queue = deque(maxlen=window_size)
imot_queue = deque(maxlen=window_size)
rpm_queue = deque(maxlen=window_size)
throttle_queue = deque(maxlen=window_size)
torque_queue = deque(maxlen=window_size)
speed_queue = deque(maxlen=window_size)
p_mech_queue = deque(maxlen=window_size)
p_mot_queue = deque(maxlen=window_size)
p_batt_queue = deque(maxlen=window_size)

# Initialize variables
soc = 0.0
discharge_time = 0.0
battery_energy = 0.0

#Connection to database
engine = create_engine('postgresql://postgres:(CIMB2023)@proyectosanti.postgres.database.azure.com:5432/postgres')
print('Connected to database')

# Connect to the serial port
ser = serial.Serial(port, baudrate, timeout=1)
ser.flush

# Read and process data
data_list = []
while True:
    try:
        # Read line from serial port
        line = ser.readline().decode().strip()
        print('Serial Read')
        
        # Split the line into individual values
        values = line.split(',')
        
        # Parse the data
        v_mot, i_mot, v_batt, i_batt, rpm, throttle, torque, speed, p_mech, p_mot, p_batt = map(float, values)
        
        # Calculate variable moving averages
        vbatt_queue.append(v_batt)
        ibatt_queue.append(i_batt)
        vmot_queue.append(v_mot)
        imot_queue.append(i_mot)
        rpm_queue.append(rpm)
        throttle_queue.append(throttle)
        torque_queue.append(torque)
        speed_queue.append(speed)
        p_mech_queue.append(p_mech)
        p_mot_queue.append(p_mot)
        p_batt_queue.append(p_batt)

        v_batt_avg = sum(vbatt_queue) / len(vbatt_queue)
        i_batt_avg = sum(ibatt_queue) / len(ibatt_queue)
        v_mot_avg = sum(vmot_queue) / len(vmot_queue)
        i_mot_avg = sum(imot_queue) / len(imot_queue)
        rpm_avg = sum(rpm_queue) / len(rpm_queue)
        throttle_avg = sum(throttle_queue) / len(throttle_queue)
        torque_avg = sum(torque_queue) / len(torque_queue)
        speed_avg = sum(speed_queue) / len(speed_queue)
        p_mech_avg = sum(p_mech_queue) / len(p_mech_queue)
        p_mot_avg = sum(p_mot_queue) / len(p_mot_queue)
        p_batt_avg = sum(p_batt_queue) / len(p_batt_queue)
        
        # Calculate state of charge (SOC)
        soc = 0.5314 * v_batt_avg**3 - 80.77 * v_batt_avg**2 + 4101.7 * v_batt_avg - 69512
        
        # Calculate discharge time
        if i_batt_avg > 0:
            discharge_time = capacity * soc / i_batt_avg
        else:
            discharge_time = float('inf')
        
        # Calculate battery energy using integral approximation
        current_time = datetime.now()
        if 'last_timestamp' not in locals():
            last_timestamp = current_time
        else:
            time_diff = current_time - last_timestamp
            elapsed_seconds = time_diff.total_seconds()
            battery_energy += (v_batt_avg * i_batt_avg * elapsed_seconds) / 3600.0  # Convert to watt-hours
            
            # Update last timestamp
            last_timestamp = current_time
        
        # Create a new data row with the datetime and all variables
        data_row = [v_mot_avg, i_mot_avg, v_batt_avg, i_batt_avg, rpm_avg, throttle_avg, torque_avg, speed_avg, p_mech_avg, p_mot_avg, p_batt_avg,
                    soc, discharge_time, battery_energy]
        data_row.insert(0,datetime.now())
        
        # Append the data row to the list
        data_list.append(data_row)

        # Create a pandas DataFrame from the data list
        columns = ['measurement_time', 'V_mot', 'I_mot', 'V_batt', 'I_batt', 'RPM', 'Throttle', 'Torque', 'Speed', 'P_mech', 'P_mot',
        'P_batt', 'SOC', 'Discharge_time', 'Battery_energy']
        then = time.time()
        df = pd.DataFrame(data_list, columns=columns)
        df.to_sql('measurements', engine, if_exists='append', index=False)
        data_list = []
        print('Data UPLOADED')
        now = time.time()-then
        print(df)
        print(now)
        
    except KeyboardInterrupt:
        break