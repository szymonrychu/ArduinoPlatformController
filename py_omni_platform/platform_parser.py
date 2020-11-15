from serial_helper import ThreadedSerialOutputHandler

class PlatformParser(ThreadedSerialOutputHandler):

    def parse_serial(self, wheel_id, raw_data):
        print(f"{wheel_id+1}: {raw_data}")