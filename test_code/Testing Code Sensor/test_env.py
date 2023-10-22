#import serial_loop_gyro_tof as serial_test
import serial_loop_gyro_tof as serial_test
serializer = serial_test.Serializer_GT()
serializer.activate()
serializer.read_ang()