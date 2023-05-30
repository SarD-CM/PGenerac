const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline');
const { DateTime } = require('luxon'); // Library for date and time operations
const { Deque } = require('collections'); // Collection class for creating queues
const { Sequelize } = require('sequelize'); // Library for interacting with databases

// Serial port configuration
const port = '/dev/ttyACM0';
const baudrate = 115200;

// Battery parameters
const nominal_voltage = 48.1;
const maximum_voltage = 54.0;
const minimum_voltage = 43.2;
const capacity = 52.0;
const nominal_energy = 2.5;

// Moving average window size (0.25 seconds at 1kHz frequency)
const window_size = 250;

// Initialize moving average queues
const vbatt_queue = new Deque([], window_size);
const ibatt_queue = new Deque([], window_size);
const vmot_queue = new Deque([], window_size);
const imot_queue = new Deque([], window_size);
const rpm_queue = new Deque([], window_size);
const throttle_queue = new Deque([], window_size);
const torque_queue = new Deque([], window_size);
const speed_queue = new Deque([], window_size);
const p_mech_queue = new Deque([], window_size);
const p_mot_queue = new Deque([], window_size);
const p_batt_queue = new Deque([], window_size);

// Initialize variables
let soc = 0.0;
let discharge_time = 0.0;
let battery_energy = 0.0;

// Connection to database
const sequelize = new Sequelize('postgres', 'postgres', '(CIMB2023)', {
  host: 'proyectosanti.postgres.database.azure.com',
  port: 5432,
  dialect: 'postgres'
});
const Measurements = sequelize.define('measurements', {
  measurement_time: {
    type: Sequelize.DATE,
    allowNull: false
  },
  V_mot: Sequelize.FLOAT,
  I_mot: Sequelize.FLOAT,
  V_batt: Sequelize.FLOAT,
  I_batt: Sequelize.FLOAT,
  RPM: Sequelize.FLOAT,
  Throttle: Sequelize.FLOAT,
  Torque: Sequelize.FLOAT,
  Speed: Sequelize.FLOAT,
  P_mech: Sequelize.FLOAT,
  P_mot: Sequelize.FLOAT,
  P_batt: Sequelize.FLOAT,
  SOC: Sequelize.FLOAT,
  Discharge_time: Sequelize.FLOAT,
  Battery_energy: Sequelize.FLOAT
});

async function connectToDatabase() {
  try {
    await sequelize.authenticate();
    console.log('Connected to database');
  } catch (error) {
    console.error('Unable to connect to the database:', error);
  }
}

connectToDatabase();

// Connect to the serial port
const portConfig = { baudRate: baudrate };
const serialPort = new SerialPort(port, portConfig);
const parser = serialPort.pipe(new Readline({ delimiter: '\r\n' }));

// Read and process data
const data_list = [];
parser.on('data', (line) => {
  try {
    console.log('Serial Read');

    // Split the line into individual values
    const values = line.split(',');

    // Parse the data
    const [
      v_mot,
      i_mot,
      v_batt,
      i_batt,
      rpm,
      throttle,
      torque,
      speed,
      p_mech,
      p_mot,
      p_batt
    ] = values.map(parseFloat);

    // Calculate variable moving averages
    vbatt_queue.push(v_batt);
    ibatt_queue.push(i_batt);
    vmot_queue.push(v_mot);
    imot_queue.push(i_mot);
    rpm_queue.push(rpm);
    throttle_queue.push(throttle);
    torque_queue.push(torque);
    speed_queue.push(speed);
    p_mech_queue.push(p_mech);
    p_mot_queue.push(p_mot);
    p_batt_queue.push(p_batt);

    const v_batt_avg = vbatt_queue.reduce((sum, value) => sum + value, 0) / vbatt_queue.length;
    const i_batt_avg = ibatt_queue.reduce((sum, value) => sum + value, 0) / ibatt_queue.length;
    const v_mot_avg = vmot_queue.reduce((sum, value) => sum + value, 0) / vmot_queue.length;
    const i_mot_avg = imot_queue.reduce((sum, value) => sum + value, 0) / imot_queue.length;
    const rpm_avg = rpm_queue.reduce((sum, value) => sum + value, 0) / rpm_queue.length;
    const throttle_avg = throttle_queue.reduce((sum, value) => sum + value, 0) / throttle_queue.length;
    const torque_avg = torque_queue.reduce((sum, value) => sum + value, 0) / torque_queue.length;
    const speed_avg = speed_queue.reduce((sum, value) => sum + value, 0) / speed_queue.length;
    const p_mech_avg = p_mech_queue.reduce((sum, value) => sum + value, 0) / p_mech_queue.length;
    const p_mot_avg = p_mot_queue.reduce((sum, value) => sum + value, 0) / p_mot_queue.length;
    const p_batt_avg = p_batt_queue.reduce((sum, value) => sum + value, 0) / p_batt_queue.length;

    // Calculate state of charge (SOC)
    soc = 0.5314 * Math.pow(v_batt_avg, 3) - 80.77 * Math.pow(v_batt_avg, 2) + 4101.7 * v_batt_avg - 69512;

    // Calculate discharge time
    if (i_batt_avg > 0) {
      discharge_time = capacity * soc / i_batt_avg;
    } else {
      discharge_time = Infinity;
    }

    // Calculate battery energy using integral approximation
    const current_time = DateTime.now();
    let last_timestamp = current_time;
    if (data_list.length > 0) {
      const time_diff = current_time.diff(last_timestamp, 'seconds');
      const elapsed_seconds = time_diff.as('seconds');
      battery_energy += (v_batt_avg * i_batt_avg * elapsed_seconds) / 3600.0; // Convert to watt-hours

      // Update last timestamp
      last_timestamp = current_time;
    }

    // Create a new data row with the datetime and all variables
    const data_row = [
      current_time.toJSDate(),
      v_mot_avg,
      i_mot_avg,
      v_batt_avg,
      i_batt_avg,
      rpm_avg,
      throttle_avg,
      torque_avg,
      speed_avg,
      p_mech_avg,
      p_mot_avg,
      p_batt_avg,
      soc,
      discharge_time,
      battery_energy
    ];

    data_list.push(data_row);

    // Create a database row with the datetime and all variables
    const measurement = {
      measurement_time: current_time.toJSDate(),
      V_mot: v_mot_avg,
      I_mot: i_mot_avg,
      V_batt: v_batt_avg,
      I_batt: i_batt_avg,
      RPM: rpm_avg,
      Throttle: throttle_avg,
      Torque: torque_avg,
      Speed: speed_avg,
      P_mech: p_mech_avg,
      P_mot: p_mot_avg,
      P_batt: p_batt_avg,
      SOC: soc,
      Discharge_time: discharge_time,
      Battery_energy: battery_energy
    };

    Measurements.create(measurement)
      .then(() => {
        console.log('Data UPLOADED');
        console.log(data_list);
      })
      .catch((error) => {
        console.error('Error uploading data:', error);
      });
  } catch (error) {
    console.error('Error processing data:', error);
  }
});

parser.on('error', (error) => {
  console.error('Serial port error:', error);
});