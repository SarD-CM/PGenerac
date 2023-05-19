from datetime import timedelta
from multiprocessing import Value
import serial

def estimate_soc_and_discharge_time(data_list):
    measurement_time_index = 0
    v_batt_index = 3
    i_batt_index = 4

    nominal_voltage = 48  # Nominal voltage of the battery (in volts)
    max_voltage = 54  # Maximum voltage of the battery (in volts)
    min_voltage = 42  # Minimum voltage of the battery (in volts)
    battery_capacity = 2.5  # Battery capacity (in kWh)

    voltage_mobile_avg = 0
    current_mobile_avg = 0
    energy_accumulated = 0

    # Open the serial port for communication
    ser = serial.Serial('/dev/ttyUSB0', 9600)  # Modify the port and baudrate as per your setup

    for data_entry in data_list:
        measurement_time = data_entry[measurement_time_index]
        v_batt = float(data_entry[v_batt_index])
        i_batt = float(data_entry[i_batt_index])

        # Calculate voltage and current moving average
        voltage_mobile_avg = (voltage_mobile_avg + v_batt) / 2
        current_mobile_avg = (current_mobile_avg + i_batt) / 2

        # Perform SOC estimation based on voltage and current average
        soc = ((voltage_mobile_avg - min_voltage) / (max_voltage - min_voltage)) * 100

        # Calculate battery energy using integral approximation
        delta_t = timedelta.total_seconds(measurement_time - data_list[0][measurement_time_index]) / 3600  # Time difference in hours
        energy = voltage_mobile_avg * current_mobile_avg * delta_t
        energy_accumulated += energy

        # Estimate remaining battery capacity
        remaining_capacity = battery_capacity - (energy_accumulated / 1000)  # Convert accumulated energy to kWh

        # Perform discharge time estimation based on remaining capacity and current average
        if current_mobile_avg != 0:
            discharge_time_hours = remaining_capacity / current_mobile_avg
            discharge_time = timedelta(hours=discharge_time_hours)
        else:
            discharge_time = timedelta.max  # If current average is 0, set discharge time to maximum

        # Determine the case scenario based on SOC
        if soc > 75:
            case_scenario = "1"  # Case 1: High SOC
        elif soc > 50:
            case_scenario = "2"  # Case 2: Medium SOC
        elif soc > 25:
            case_scenario = "3"  # Case 3: Low SOC
        else:
            case_scenario = "4"  # Case 4: Critical SOC

        # Append SOC, discharge time, remaining capacity, and case scenario to the data_entry sublist
        data_entry.append(soc)
        data_entry.append(discharge_time)
        data_entry.append(remaining_capacity)
        data_entry.append(case_scenario)

        # Print the SOC, discharge time, remaining capacity, and case scenario for verification
        print(f"SOC: {soc}%\tDischarge Time: {discharge_time}\tRemaining Capacity: {remaining_capacity} kWh\tCase: {case_scenario}")

        # Send the case scenario through the serial port
        ser.write(case_scenario.encode())  # Encode the string and send it as bytes

    # Close the serial port
    ser.close()

if __name__ == '__main__':
    data_list = Value('i', [])  # Shared variable to hold the list of lists
    estimate_soc
