import pandas as pd
from sqlalchemy import create_engine
from multiprocessing import Value

def upload_to_postgresql(data_list):
    measurement_time_index = 0
    v_mot_index = 1
    i_mot_index = 2
    b_batt_index = 3
    i_batt_index = 4
    rpm_index = 5
    throttle_index = 6
    torque_index = 7
    speed_index = 8
    soc_index = 9
    discharge_time_index = 10
    energy_index = 11

    # Create a DataFrame from the data_list
    columns = ['Measurement Time', 'V_mot', 'I_mot', 'B_batt', 'I_batt', 'RPM', 'Throttle', 'Torque', 'Speed', 'SOC', 'Discharge Time', 'Energy']
    df = pd.DataFrame(data_list, columns=columns)

    # Create a connection to the PostgreSQL database
    engine = create_engine('postgresql://username:password@localhost:5432/database_name')

    # Upload the DataFrame to the PostgreSQL database
    df.to_sql('table_name', engine, if_exists='append', index=False)

if __name__ == '__main__':
    data_list = Value('i', [])  # Shared variable to hold the list of lists

    while True:
        if len(data_list) > 0:
            upload_to_postgresql(data_list)
            data_list.value = []  # Clear the data_list after upload

        time.sleep(1)  # Delay for 1 second before next upload
