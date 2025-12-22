const express = require('express');
const WebSocket = require('ws');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

const app = express();
const PORT = 3000;

// Serve static files
app.use(express.static('.'));

const server = app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
  console.log('Open index.html in your browser');
});

// WebSocket server
const wss = new WebSocket.Server({ server });

// Serial port configuration
const SERIAL_PORT = '/dev/tty.usbmodem1103'; // Update this to your port
const BAUD_RATE = 115200;

let serialPort = null;
let parser = null;

// Initialize serial port
function initSerial() {
  try {
    serialPort = new SerialPort({
      path: SERIAL_PORT,
      baudRate: BAUD_RATE,
    });

    parser = serialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

    serialPort.on('open', () => {
      console.log(`Serial port ${SERIAL_PORT} opened at ${BAUD_RATE} baud`);
    });

    parser.on('data', (line) => {
      console.log('Received:', line);  // DEBUG
      // Broadcast to all connected WebSocket clients
      wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
          client.send(line);
        }
      });
    });

    serialPort.on('error', (err) => {
      console.error('Serial port error:', err.message);
    });

  } catch (err) {
    console.error('Failed to open serial port:', err.message);
    console.log('Running in demo mode - no serial data');
  }
}

// WebSocket connections
wss.on('connection', (ws) => {
  console.log('WebSocket client connected');

  ws.on('close', () => {
    console.log('WebSocket client disconnected');
  });

  ws.on('error', (err) => {
    console.error('WebSocket error:', err.message);
  });
});

// Initialize serial when server starts
initSerial();

// Graceful shutdown
process.on('SIGINT', () => {
  console.log('\nClosing connections...');
  if (serialPort && serialPort.isOpen) {
    serialPort.close();
  }
  process.exit(0);
});
