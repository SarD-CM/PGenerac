import serial
import pandas as pd
from sqlalchemy import create_engine
from datetime import datetime

data_list = []

engine = create_engine('postgresql://postgres:(CIMB2023)@proyectosanti.postgres.database.azure.com:5432/postgres')
print('Connected to database')

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush

while True:
    data = ser.readline().decode().strip() 
    if data != '':
        data_split = data.split(",")
        data_split.insert(0, datetime.now())
        data_list.append(data_split)
        df = pd.DataFrame(data_list,columns=['measurement_time','V_mot','I_mot','V_batt','I_batt','RPM','Throttle','Torque','Speed','P_mech','P_mot','P_batt','SOC','Discharge_time'])
        df.to_sql('measurements', engine, if_exists='append', index=False)
        data_list = []
        print('Data UPLOADED')
        print(df)