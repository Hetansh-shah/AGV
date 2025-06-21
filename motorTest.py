import serial
import time
import sys
import os


class IWSServoMotor:
    def __init__(self, port='/dev/ttyACM0', motor_id=1):
        """Initialize the IWS servo hub motor controller."""
        self.port = port
        self.motor_id = motor_id
        self.ser = None

    def open_connection(self):
        """Open the serial connection to the motor."""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )

            # Clear buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            print(f"Connected to motor on {self.port}")
            return True
        except Exception as e:
            print(f"Error opening port {self.port}: {e}")
            return False

    def close_connection(self):
        """Close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Disconnected from {self.port}")

    def send_command(self, hex_command):
        """Send a hex command to the motor and receive the response."""
        if not self.ser or not self.ser.is_open:
            if not self.open_connection():
                return None

        try:
            # Convert hex string to bytes if it's a string
            if isinstance(hex_command, str):
                cmd = bytes.fromhex(hex_command)
            else:
                cmd = hex_command

            # Clear buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # Send command
            self.ser.write(cmd)
            self.ser.flush()

            # Wait for response
            time.sleep(0.1)

            # Read response
            response = bytearray()
            start_time = time.time()

            while time.time() - start_time < 0.5:  # 500ms timeout
                if self.ser.in_waiting > 0:
                    response.extend(self.ser.read(self.ser.in_waiting))
                    if len(response) >= 10:  # We have at least one full response
                        break
                time.sleep(0.01)  # Wait 10ms between checks

            return response if response else None

        except Exception as e:
            print(f"Communication error: {e}")
            return None

    def calculate_checksum(self, data):
        """Calculate the checksum for a command."""
        return sum(data) & 0xFF

    def build_command(self, cmd, addr, data, err_r=0):
        """Build a command with proper format and checksum."""
        addr_h = (addr >> 8) & 0xFF
        addr_l = addr & 0xFF

        # Build command array
        command = [self.motor_id, cmd, addr_h, addr_l, err_r]
        command.extend(data)

        # Add checksum
        checksum = self.calculate_checksum(command)
        command.append(checksum)

        return bytes(command)

    def read_data(self, addr, data_size=2, verbose=False):
        """Read data from the motor at the specified address."""
        # Create read command
        cmd = self.build_command(0xA0, addr, [0, 0, 0, 0])

        if verbose:
            print(f"Sending: {' '.join(f'0x{b:02X}' for b in cmd)}")

        # Send command and get response
        response = self.send_command(cmd)

        if not response or len(response) < 10:
            if verbose:
                print("No valid response received")
            return None

        if verbose:
            print(f"Received: {' '.join(f'0x{b:02X}' for b in response)}")

        # Check if response indicates error
        if response[1] in [0x5F, 0x50, 0x58, 0x80]:
            error_msg = {
                0x5F: "Object does not exist",
                0x50: "Data length is wrong",
                0x58: "Object address is not writable",
                0x80: "Check is wrong"
            }.get(response[1], f"Unknown error (0x{response[1]:02X})")

            if verbose:
                print(f"Error: {error_msg}")
            return None

        # Extract and return the data based on size
        data = response[5:9]

        if data_size == 1:  # 8-bit
            value = data[3]
            # Handle signed values
            if value > 127:
                value = value - 256
        elif data_size == 2:  # 16-bit
            value = (data[2] << 8) | data[3]
            # Handle signed values
            if value > 32767:
                value = value - 65536
        elif data_size == 4:  # 32-bit
            value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
            # Handle signed values
            if value > 2147483647:
                value = value - 4294967296
        else:
            if verbose:
                print(f"Invalid data size: {data_size}")
            return None

        return value

    def write_data(self, addr, value, data_size=2, verbose=False):
        """Write data to the motor at the specified address."""
        # Handle negative values
        if value < 0:
            if data_size == 1:
                value = 256 + value  # Convert to 8-bit unsigned
            elif data_size == 2:
                value = 65536 + value  # Convert to 16-bit unsigned
            elif data_size == 4:
                value = 4294967296 + value  # Convert to 32-bit unsigned

        # Prepare command based on data size
        if data_size == 1:  # 8-bit
            cmd_type = 0x51
            data = [0, 0, 0, value & 0xFF]
        elif data_size == 2:  # 16-bit
            cmd_type = 0x52
            data = [0, 0, (value >> 8) & 0xFF, value & 0xFF]
        elif data_size == 4:  # 32-bit
            cmd_type = 0x54
            data = [(value >> 24) & 0xFF, (value >> 16) & 0xFF,
                    (value >> 8) & 0xFF, value & 0xFF]
        else:
            if verbose:
                print(f"Invalid data size: {data_size}")
            return False

        # Create write command
        cmd = self.build_command(cmd_type, addr, data)

        if verbose:
            print(f"Sending: {' '.join(f'0x{b:02X}' for b in cmd)}")

        # Send command and get response
        response = self.send_command(cmd)

        if not response or len(response) < 10:
            if verbose:
                print("No valid response received")
            return False

        if verbose:
            print(f"Received: {' '.join(f'0x{b:02X}' for b in response)}")

        # Check if response indicates success
        expected_resp = {
            0x51: 0x61,  # 8-bit write
            0x52: 0x62,  # 16-bit write
            0x54: 0x64  # 32-bit write
        }

        if response[1] != expected_resp.get(cmd_type):
            error_msg = {
                0x5F: "Object does not exist",
                0x50: "Data length is wrong",
                0x58: "Object address is not writable",
                0x80: "Check is wrong"
            }.get(response[1], f"Unknown error (0x{response[1]:02X})")

            if verbose:
                print(f"Error: {error_msg}")
            return False

        return True

    # High-level motor control functions

    def get_status(self, verbose=True):
        """Get the motor's status."""
        status = {}

        # Read driver status word
        status_word = self.read_data(0x7001, 2, verbose=False)
        if status_word is not None:
            status["status_word"] = f"0x{status_word:04X}"
            status["driver_normal"] = (status_word == 0)
            status["driver_error"] = bool(status_word & 0x0008)

        # Read error code
        error_code = self.read_data(0x7011, 2, verbose=False)
        if error_code is not None:
            status["error_code"] = f"0x{error_code:04X}"

            # Decode error bits
            error_bits = {
                0: "Internal error",
                1: "Encoder ABZ signal error",
                2: "Encoder UVW signal error",
                3: "Encoder counting error",
                4: "Driver temperature too high",
                5: "Driver bus voltage too high",
                6: "Driver bus voltage too low",
                7: "Driver output short-circuit",
                8: "Braking resistor temperature too high",
                9: "Following error over-range",
                11: "I²*T error (Overload)",
                12: "Speed following error over-range",
                13: "Motor temperature too high",
                14: "Searching motor failed",
                15: "Communication failed"
            }

            active_errors = []
            for bit, desc in error_bits.items():
                if error_code & (1 << bit):
                    active_errors.append(desc)

            status["active_errors"] = active_errors

        # Read operation mode
        mode = self.read_data(0x7018, 1, verbose=False)
        if mode is not None:
            mode_desc = {
                1: "Position mode",
                3: "Speed mode with Acc/Dec",
                -3: "Speed mode without Acc/Dec",
                4: "Torque mode"
            }
            status["operation_mode"] = mode_desc.get(mode, f"Unknown mode: {mode}")

        # Read actual position
        position = self.read_data(0x7071, 4, verbose=False)
        if position is not None:
            status["actual_position"] = position

        # Read actual speed
        speed = self.read_data(0x7075, 2, verbose=False)
        if speed is not None:
            status["actual_speed_rpm"] = speed

        # Read motor temperature
        temp = self.read_data(0x7002, 2, verbose=False)
        if temp is not None:
            status["driver_temperature"] = temp

        # Read bus voltage
        voltage = self.read_data(0x5001, 2, verbose=False)
        if voltage is not None:
            status["bus_voltage"] = voltage

        if verbose:
            print("\nMotor Status:")
            for key, value in status.items():
                print(f"  {key}: {value}")

        return status

    def clear_errors(self):
        """Clear any errors in the motor."""
        print("Clearing errors...")
        return self.write_data(0x7019, 0x86, 2)

    def enable_motor(self):
        """Enable the motor."""
        print("Enabling motor...")
        return self.write_data(0x7019, 0x0F, 2)

    def disable_motor(self):
        """Disable the motor."""
        print("Disabling motor...")
        return self.write_data(0x7019, 0x06, 2)

    def set_operation_mode(self, mode):
        """Set the operation mode.

        Args:
            mode (int): 1 for position mode, 3 for speed mode with Acc/Dec,
                      -3 for speed mode without Acc/Dec, 4 for torque mode
        """
        mode_name = {
            1: "Position mode",
            3: "Speed mode with Acc/Dec",
            -3: "Speed mode without Acc/Dec",
            4: "Torque mode"
        }.get(mode, f"Mode {mode}")

        print(f"Setting operation mode to {mode_name}...")
        return self.write_data(0x7017, mode, 1)

    def set_acceleration(self, acc_rps2):
        """Set acceleration in rps/s (rotations per second squared)."""
        # Convert to internal DEC value
        # Formula: [DEC]=[rps/s]*256*[Resolution]/15625
        resolution = 4096  # Default encoder resolution
        dec_value = int((acc_rps2 * 256 * resolution) / 15625)
        print(f"Setting acceleration to {acc_rps2} rps/s (DEC value: {dec_value})...")
        return self.write_data(0x7099, dec_value, 4)

    def set_deceleration(self, dec_rps2):
        """Set deceleration in rps/s (rotations per second squared)."""
        # Convert to internal DEC value
        resolution = 4096  # Default encoder resolution
        dec_value = int((dec_rps2 * 256 * resolution) / 15625)
        print(f"Setting deceleration to {dec_rps2} rps/s (DEC value: {dec_value})...")
        return self.write_data(0x709A, dec_value, 4)

    def set_speed(self, rpm):
        """Set target velocity in rpm for speed mode."""
        print(f"Setting target velocity to {rpm} rpm...")
        return self.write_data(0x70B1, rpm, 2)

    def move_to_position(self, position, is_absolute=True):
        """Move to a target position."""
        pos_type = "absolute" if is_absolute else "relative"
        print(f"Moving to {pos_type} position {position}...")

        # Set target position
        addr = 0x7091 if is_absolute else 0x709F
        success = self.write_data(addr, position, 4)
        if not success:
            print("Failed to set target position")
            return False

        # Set control word to start motion
        control_word = 0x1F if is_absolute else 0x0F
        return self.write_data(0x7019, control_word, 2)

    def set_profile_velocity(self, rpm):
        """Set profile velocity in rpm for position mode."""
        print(f"Setting profile velocity to {rpm} rpm...")
        return self.write_data(0x709D, rpm, 2)

    def initialize(self):
        """Initialize the motor with safe settings."""
        print("Initializing motor...")

        # Clear any errors
        self.clear_errors()
        time.sleep(0.1)

        # Set reasonable acceleration/deceleration
        self.set_acceleration(2.0)  # 2 rps/s
        self.set_deceleration(2.0)  # 2 rps/s
        time.sleep(0.1)

        # Enable the motor
        success = self.enable_motor()
        if success:
            print("Motor initialized successfully")
        else:
            print("Failed to initialize motor")

        return success

    def run_sequence(self, sequence):
        """Run a predefined sequence of commands."""
        if sequence == "position_test":
            print("Running position test sequence...")

            # Initialize
            self.clear_errors()
            time.sleep(0.1)

            # Set to position mode
            self.set_operation_mode(1)
            time.sleep(0.1)

            # Set acceleration/deceleration
            self.set_acceleration(2.0)
            self.set_deceleration(2.0)
            time.sleep(0.1)

            # Set profile velocity
            self.set_profile_velocity(100)
            time.sleep(0.1)

            # Enable motor
            self.enable_motor()
            time.sleep(0.5)

            # Move to position 1000
            self.move_to_position(1000, True)
            print("Moving to position 1000...")
            time.sleep(3)

            # Move to position 0
            self.move_to_position(0, True)
            print("Moving to position 0...")
            time.sleep(3)

            # Disable motor
            self.disable_motor()

            print("Position test sequence completed")

        elif sequence == "speed_test":
            print("Running speed test sequence...")

            # Initialize
            self.clear_errors()
            time.sleep(0.1)

            # Set to speed mode
            self.set_operation_mode(3)
            time.sleep(0.1)

            # Set acceleration/deceleration
            self.set_acceleration(2.0)
            self.set_deceleration(2.0)
            time.sleep(0.1)

            # Enable motor
            self.enable_motor()
            time.sleep(0.5)

            # Set speed to 50 rpm
            self.set_speed(50)
            print("Running at 50 rpm...")
            time.sleep(3)

            # Set speed to -50 rpm (opposite direction)
            self.set_speed(-50)
            print("Running at -50 rpm...")
            time.sleep(3)

            # Stop
            self.set_speed(0)
            print("Stopping...")
            time.sleep(1)

            # Disable motor
            self.disable_motor()

            print("Speed test sequence completed")

        else:
            print(f"Unknown sequence: {sequence}")
            return False

        return True


def clear_screen():
    """Clear the console screen."""
    os.system('cls' if os.name == 'nt' else 'clear')


def print_menu():
    """Print the main menu."""
    clear_screen()
    print("=" * 60)
    print("IWS SERIES SERVO HUB MOTOR CONTROLLER")
    print("=" * 60)
    print("Available Commands:")
    print("-" * 60)
    print("  1. Get motor status")
    print("  2. Clear errors")
    print("  3. Initialize motor (clear errors, set acc/dec, enable)")
    print("  4. Enable motor")
    print("  5. Disable motor")
    print("  6. Set operation mode")
    print("  7. Set speed")
    print("  8. Move to position")
    print("  9. Set acceleration/deceleration")
    print(" 10. Run test sequence")
    print(" 11. Send custom command")
    print(" 12. Exit")
    print("-" * 60)


def get_operation_mode_menu():
    """Get the operation mode from the user."""
    clear_screen()
    print("=" * 60)
    print("SET OPERATION MODE")
    print("=" * 60)
    print("Available Modes:")
    print("-" * 60)
    print("  1. Position mode")
    print("  2. Speed mode with acceleration/deceleration")
    print("  3. Speed mode without acceleration/deceleration")
    print("  4. Torque mode")
    print("-" * 60)

    mode_map = {
        "1": 1,  # Position mode
        "2": 3,  # Speed mode with Acc/Dec
        "3": -3,  # Speed mode without Acc/Dec
        "4": 4  # Torque mode
    }

    choice = input("Enter your choice [1-4]: ")

    if choice in mode_map:
        return mode_map[choice]
    else:
        print("Invalid choice. Using position mode.")
        return 1


def get_test_sequence_menu():
    """Get the test sequence from the user."""
    clear_screen()
    print("=" * 60)
    print("RUN TEST SEQUENCE")
    print("=" * 60)
    print("Available Sequences:")
    print("-" * 60)
    print("  1. Position test sequence")
    print("  2. Speed test sequence")
    print("-" * 60)

    choice = input("Enter your choice [1-2]: ")

    if choice == "1":
        return "position_test"
    elif choice == "2":
        return "speed_test"
    else:
        print("Invalid choice. Running position test.")
        return "position_test"


def main():
    port = '/dev/ttyACM0'  # Default port

    # Try to get the port from command line
    if len(sys.argv) > 1:
        port = sys.argv[1]

    # Create motor control object
    motor = IWSServoMotor(port)

    # Open connection
    if not motor.open_connection():
        input("Press Enter to exit...")
        return

    try:
        while True:
            print_menu()

            choice = input("Enter your choice [1-12]: ")

            if choice == "1":
                # Get motor status
                motor.get_status()

            elif choice == "2":
                # Clear errors
                motor.clear_errors()

            elif choice == "3":
                # Initialize motor
                motor.initialize()

            elif choice == "4":
                # Enable motor
                motor.enable_motor()

            elif choice == "5":
                # Disable motor
                motor.disable_motor()

            elif choice == "6":
                # Set operation mode
                mode = get_operation_mode_menu()
                motor.set_operation_mode(mode)

            elif choice == "7":
                # Set speed
                try:
                    clear_screen()
                    print("SET SPEED")
                    print("-" * 60)
                    rpm = int(input("Enter speed in RPM (negative for reverse): "))

                    # Make sure we're in speed mode
                    motor.set_operation_mode(3)
                    time.sleep(0.1)

                    # Set speed
                    motor.set_speed(rpm)
                except ValueError:
                    print("Invalid input. Please enter a number.")

            elif choice == "8":
                # Move to position
                try:
                    clear_screen()
                    print("MOVE TO POSITION")
                    print("-" * 60)
                    position = int(input("Enter target position: "))
                    mode = input("Absolute (a) or Relative (r) positioning? [a/r]: ").lower()
                    is_absolute = mode != 'r'

                    # Make sure we're in position mode
                    motor.set_operation_mode(1)
                    time.sleep(0.1)

                    # Set profile velocity if not using relative mode
                    if is_absolute:
                        motor.set_profile_velocity(100)
                        time.sleep(0.1)

                    # Move to position
                    motor.move_to_position(position, is_absolute)
                except ValueError:
                    print("Invalid input. Please enter a number.")

            elif choice == "9":
                # Set acceleration/deceleration
                try:
                    clear_screen()
                    print("SET ACCELERATION/DECELERATION")
                    print("-" * 60)
                    acc = float(input("Enter acceleration in rps/s [1-10]: "))
                    dec = float(input("Enter deceleration in rps/s [1-10]: "))

                    motor.set_acceleration(acc)
                    motor.set_deceleration(dec)
                except ValueError:
                    print("Invalid input. Please enter a number.")

            elif choice == "10":
                # Run test sequence
                sequence = get_test_sequence_menu()
                motor.run_sequence(sequence)

            elif choice == "11":
                # Send custom command
                clear_screen()
                print("SEND CUSTOM COMMAND")
                print("-" * 60)
                print("Enter hex command (e.g., 01A07001000000000012)")
                hex_cmd = input("Command: ")

                try:
                    response = motor.send_command(hex_cmd)
                    if response:
                        print(f"Received: {' '.join(f'0x{b:02X}' for b in response)}")
                    else:
                        print("No response received")
                except Exception as e:
                    print(f"Error: {e}")

            elif choice == "12":
                # Exit
                break

            else:
                print("Invalid choice. Please try again.")

            # Pause after each command
            input("\nPress Enter to continue...")

    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Always close the connection
        motor.close_connection()

        # Final pause
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
