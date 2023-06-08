import serial
import pandas as pd
from sqlalchemy import create_engine
from datetime import datetime

data_list = []

engine = create_engine('postgresql://postgres:(CIMB2023)@proyectosanti.postgres.database.azure.com:5432/postgres')
print('Connected to database')

ser = serial.Serial('/dev/ttyACM4', 115200, timeout=1)
ser.flush()
ser.reset_input_buffer()
ser.reset_output_buffer()

while True:
    try:
        data = ser.readline().decode('utf-8').strip()
        now = datetime.now()
        cond = 0
        print('serial read')
        print(now)
        if data != '':
            data_split = data.split(",")
            data_split.insert(0,cond)
            data_split.insert(0, now)
            data_list.append(data_split)
            print(len(data_list[0]))
            if len(data_list[0]) == 15:
                df = pd.DataFrame(data_list,columns=['measurement_time','Condition','V_mot','I_mot','V_batt','I_batt','rpm','Throttle','Torque','Speed','P_mech','P_mot','P_batt','SOC','Discharge_time'])
                df.to_sql('measurements', engine, if_exists='append', index=False)
                print(df)
                print('Data UPLOADED\n')
                data_list = []
            else:
                print('No furula\n')
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        ser.close()
