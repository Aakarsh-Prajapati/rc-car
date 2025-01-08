import serial
import time

# Serial port configuration
# Change this to your port (e.g., 'COM3' on Windows)
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# Initialize Serial Connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error: {e}")
    exit()


def send_data(vel_x, vel_w):
    """
    Send data to ESP8266 in the format 'x-10|w-12'
    """
    try:
        command = f"|x{vel_x}|w{vel_w}|\n"
        ser.write(command.encode('utf-8'))
        print(f"Sent: {command.strip()}")
    except Exception as e:
        print(f"Failed to send data: {e}")


def main():
    try:
        while True:
            # Example data to send
            vel_x = "2.6"
            vel_w = "7"

            try:
                vel_x = float(vel_x)
                vel_w = float(vel_w)
                send_data(vel_x, vel_w)
            except ValueError:
                print("Invalid input. Please enter numeric values for vel_x and vel_w.")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nScript terminated by user.")

    finally:
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
