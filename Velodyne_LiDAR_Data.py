"""
CSDN:
https://www.baidu.com/link?url=jU4TA15Xlz0L94t5d0kHiZ_DHj-PSNEIdKJbGuFg9tjYsBZF9_4U5lwzBYAe7hQbROu4tDhtZDXiermzM1PXGPdnIU2NYQH5UO60kc7YQP3&wd=&eqid=aba00c9601a461350000000565839d0a


Velodyne Manual:
https://velodynelidar.com/wp-content/uploads/2019/12/63-9243-Rev-E-VLP-16-User-Manual.pdf

"""
# Class to handle or decode lidar long bytes_array
import binascii
import logging
import math

import numpy as np


# from abc import abstractmethod


def reserve_bytes_return_decimal(data_bytes):
    MIN_LEN_BYTES = 2
    MAX_LEN_BYTES = 5
    assert MIN_LEN_BYTES <= len(data_bytes) <= MAX_LEN_BYTES, \
        f"data_bytes invalid in reserve_bytes method valid range = {[MIN_LEN_BYTES, MAX_LEN_BYTES]}"
    num = 0
    for i, byte in enumerate(data_bytes):
        num += byte << (i * 8)
    return num


def convert_Bytearray_2_Hex_str(data_bytes):
    hex_data_received = str(binascii.hexlify(data_bytes))
    return hex_data_received


def convert_Bytearray_2_int(data_bytes):
    """
    	byteorder
		The byte order used to represent the integer.  If byteorder is 'big',
		the most significant byte is at the beginning of the byte array.  If
		byteorder is 'little', the most significant byte is at the end of the
		byte array.  To request the native byte order of the host system, use
		`sys.byteorder' as the byte order value.
    """
    int_value = int.from_bytes(data_bytes, byteorder='big')
    return int_value


class Velodyne_LiDAR_Decode_Base():  # Base Class
    BLOCK_SIZE = 100
    LEN_TIMESTAMP_AND_SENSORTYPE = 6
    LEN_DATA_BYTES = 1206
    LEN_DATA_BLOCK = 1200

    def __init__(self, data_bytes):
        logging.INFO("Entered Velodyne_LiDAR_Decode_Base. __init__()")
        self.blocks = []
        self.Azimuth_angle_list = []
        self.azimuth_increment_per_half_data_block = None
        self.azimuth_increment_count = 0
        self.lines = None

        # # check input validity by  last 6 bytes.
        # assert len(data_bytes) % self.BLOCK_SIZE == self.LEN_TIMESTAMP_AND_SENSORTYPE, \
        #     "Input data length is not a multiple fo block size "

        # check input validity by  last 6 bytes.
        assert len(data_bytes) == self.LEN_DATA_BYTES, \
            "Input data length is not valid."

        # split data into blocks:
        self.blocks = [data_bytes[i:i + self.BLOCK_SIZE] for i in
                       range(0, len(data_bytes), self.BLOCK_SIZE)]
        # logging.debug(f"self.blocks = {self.blocks}")
        # logging.debug(f"len(self.blocks) = {len(self.blocks)}")

        # Calculate data block base on Flag 0xFFEE

        self.calculate_azimuth_angle_list()
        self.calculate_lidar_lines()
        self.check_single_or_dual_mode()
        logging.debug("Exiting Velodyne_LiDAR_Decode_Base. __init__()")

    def check_single_or_dual_mode(self):
        Dict_Velodyne_Sensor_Return_Type = {
            0x37: "SINGLE_MODE",
            0x38: "SINGLE_MODE",
            0x39: "DUAL_MODE"
        }
        logging.debug(f"self.blocks[-1][-2] = {self.blocks[-1][-2]}")
        raw_return_mode = self.blocks[-1][-2]  # Consult product manual
        self.str_return_mode = Dict_Velodyne_Sensor_Return_Type[raw_return_mode]
        logging.debug(f"Lidar sensor return mode '{self.str_return_mode}' lines")

    def get_sensor_type(self):
        Dict_Velodyne_SENSOR_TYPE = {
            0x21: "HDL- 32E",
            0x22: "VLP-16 or PUCK-LITE",
            0x24: "PUCK HI-RES",
            0x28: "VLP-32C",
            0x31: "VELARRARY",
            0xA1: "VLS-128"
        }
        # self.product_type = Dict_Velodyne_SENSOR_TYPE[self.blocks[-1][-1]]  # product ID offset in packet 0x4DF

        # Last byte in a message is sensor type (hardwrae)
        product_id_int = self.blocks[-1][-1]  # python will automatically return int
        return product_id_int

    def get_time_stamp(self):
        """
        Time stamp is calculate from UDP last few bytes,
        Last + second last are for return mode, and sensor type
        last Third  --- sixth are time stamp bytes
        """
        raw_time_stamp_bytes = self.blocks[-1][-6:-2]
        raw_time_stamp_decimal = reserve_bytes_return_decimal(raw_time_stamp_bytes)
        time_stamp = raw_time_stamp_decimal / 1000000  # Time stamp unit is ms, one thousandth of a second. 1/1000 Second
        return time_stamp

    def get_return_mode(self):
        Dict_Velodyne_RETURN_MODE = {
            0x37: "Strongest",
            0x38: "Last Return",
            0x39: "Dual Return"
        }

        return_mode_int = self.blocks[-1][-2]
        logging.debug(Dict_Velodyne_RETURN_MODE[return_mode_int])

        return return_mode_int

    def get_distances_and_intensities(self):
        """
        For each data point, 3 bytes data:
        2 bytes distance(unsigned integer), 1 byte reflectivity;

        distance:
        2mm granularity; e.g. 51,154 = 102,308 mm = 102.308m
            0x00 0x00 means non-measurement, not returned in time.

        """
        return 1


# class Velodyne_LiDAR_Decode_Single_Return(Velodyne_LiDAR_Decode_Base):
#     def __init__(self, data_bytes):
#         super().__init__(data_bytes)
#         print("Class Velodyne_LiDAR_Decode_Single_Return constructor inited.")
#
#
# class Velodyne_LiDAR_Decode_Dual_Return(Velodyne_LiDAR_Decode_Base):
#     def __init__(self, data_bytes):
#         super().__init__(data_bytes)
#         print("Class Velodyne_LiDAR_Decode_Dual_Return constructor init completed.")


# class Velodyne_LiDar_16_Lines(Velodyne_LiDAR_Decode_Base):
#     def __init__(self, data_bytes):
#         super().__init__(data_bytes)
#         self.total_azimuth_angle_increase = None
#         self.azimuth_increment = None
#         self.azimuth_increment_count_for_single_mode = 22  # data from user manual
#         self.azimuth_increment_count_for_dual_mode = 11  # data from user manual
#         # only apply to 16 lines, too few line,
#         # per data block has two fire sequence,
#         # angle increment calculation is necessary,
#         # Otherwise, two set of point clouds will on the same spot, wasting computation.
#         self.total_azimuth_angle_increase = self.calculate_azimuth_angle_increase_in_one_data_bytes()
#         self.azimuth_increment = self.calculate_azimuth_angle_increment()
#         self.azimuth_angle_list = self.calculate_aizmuth_angle_list()
#         print("Class Velodyne_LiDar_16_Lines constructor init completed.")


class Veldoyne_LiDAR_VLP16_Strongest():
    BLOCK_SIZE = 100
    LEN_TIMESTAMP_AND_SENSORTYPE = 6
    LEN_DATA_BYTES = 1206
    LEN_DATA_BLOCK_IN_BYTES = 1200
    LEN_DATA_BLOCK = 12
    VERTICAL_ANGLE_LIST = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
    VERTICAL_CORRECTION_LIST = [11.2, -0.7, 9.7, -2.2, 8.1, -3.7, 6.6, -5.1, 5.1, -6.6, 3.7, -8.1, 2.2, -9.7, 0.7,
                                -11.2]
    CHANNEL_DATA_LENGTH = 3

    def __init__(self, data_bytes):
        logging.debug("Entered __init__ of Veldoyne_LiDAR_VLP16_Strongest")
        self.blocks = [data_bytes[i:i + self.BLOCK_SIZE] for i in range(0, len(data_bytes), self.BLOCK_SIZE)]
        self.data_block_size = self.calculate_data_block_size()
        self.str_return_mode = None
        self.azimuth_angle_list = []
        self.point_cloud_numpy = self.init_point_cloud_numpy()
        self.calculate_default_aizmuth_angle_list()
        self.lidar_lines = self.calculate_lidar_lines()

        # data_bytes into data_blocks

        # self.print_blocks_in_bytes()
        self.total_azimuth_angle_increase = None
        self.azimuth_increment = None
        self.azimuth_increment_count_for_single_mode = 22  # data from user manual
        self.azimuth_increment_count_for_dual_mode = 11  # data from user manual
        # only apply to 16 lines, too few line,
        # per data block has two fire sequence,
        # angle increment calculation is necessary,
        # Otherwise, two set of point clouds will on the same spot, wasting computation.
        self.total_azimuth_angle_increase = self.calculate_azimuth_angle_increase_in_one_data_bytes()
        self.azimuth_increment = self.calculate_azimuth_angle_increment()  # in each datablock, include two firing sequencey, calculate angle difference between the two.
        self.azimuth_angle_list = self.add_angle_increment()
        self.process_channel_data()  # only for 16 lines, if any model is used, update needed.

        logging.debug("Exiting __init__ of Veldoyne_LiDAR_VLP16_Strongest")

    def process_channel_data(self):
        """
        # only for 16 lines, if any model is used, update needed.
        # break down the data in detail. Most code is in this method.
        """

        FIRING_SEQUENCES_IN_DATA_BLOCK = 2
        LEN_RADIUS_BYTES = 2
        LEN_REFLECTIVITY_BYTES = 1
        START_INDEX_FIRST_FIRING_SEQUENCE_FOR_CHANNEL_DATA = 4
        END_INDEX_FIRST_FIRING_SEQUENCE_FOR_CHANNEL_DATA = 51
        START_INDEX_SECOND_FIRING_SEQUENCE_FOR_CHANNEL_DATA = 52
        END_INDEX_SECOND_FIRING_SEQUENCE_FOR_CHANNEL_DATA = 99

        start_index_list = [START_INDEX_FIRST_FIRING_SEQUENCE_FOR_CHANNEL_DATA,
                            START_INDEX_SECOND_FIRING_SEQUENCE_FOR_CHANNEL_DATA]
        end_index_list = [END_INDEX_FIRST_FIRING_SEQUENCE_FOR_CHANNEL_DATA,
                          END_INDEX_SECOND_FIRING_SEQUENCE_FOR_CHANNEL_DATA]

        for i in range(self.data_block_size):  # data block index
            for j in range(FIRING_SEQUENCES_IN_DATA_BLOCK):  # first half or second half
                a_azimuth_angle = self.azimuth_angle_list[i + j]
                # logging.debug(f"data block[{i}], firing sequence[{j}]; angle_data = {azimuth_angle_double}, bytes = {channel_data_bytes}")
                for index_line in range(self.lidar_lines):
                    channel_data_start_index = start_index_list[j] + self.CHANNEL_DATA_LENGTH * index_line
                    channel_data_bytes = self.blocks[i][
                                         channel_data_start_index:channel_data_start_index + self.CHANNEL_DATA_LENGTH]
                    radius, reflectivity = self.process_one_channel_data_return_radius_nd_reflectivity(
                        channel_data_bytes)
                    W_elevation_angle = self.VERTICAL_ANGLE_LIST[index_line]
                    self.print_channel_data_log(i, j, index_line, channel_data_start_index, channel_data_bytes, radius,
                                                reflectivity)
                    x, y, z = self.convert_Spherial_2_Cartesian_then_vertical_correction(radius, a_azimuth_angle,
                                                                                         W_elevation_angle, index_line)
                    self.add_2_point_cloud_numpy(x, y, z, reflectivity)
        logging.info(f"Point cloud numpy array = {self.point_cloud_numpy}")

    def add_2_point_cloud_numpy(self, x, y, z, reflectivity):
        if x is None or y is None or z is None:
            logging.debug("Dummy Point found.")
        else:
            logging.debug("Valid point found.")
            point = [x, y, z, reflectivity]
            # Convert the point to a numpy array
            point_array = np.array(point)
            logging.info(point_array)
            # Append the point array to the point cloud array
            self.point_cloud_numpy = np.append(self.point_cloud_numpy, [point_array], axis=0)
            logging.info(f"point_cloud_numpy = {self.point_cloud_numpy}")

    def convert_Spherial_2_Cartesian_then_vertical_correction(self, radius, a_azimuth_angle, W_elevation_angle,
                                                              index_line):
        x, y, z = convert_Spherial_2_Cartesian(radius, a_azimuth_angle, W_elevation_angle)
        if x is None or y is None or z is None:
            logging.debug(f"Entered x or y or z is None branch, x= {x}, y={y},z={z} ")
            return None, None, None
        else:
            z = self.vertical_correction(z, index_line)
            return x, y, z

    def vertical_correction(self, z, index_line):
        z = z + self.VERTICAL_CORRECTION_LIST[index_line]
        return z

    def print_channel_data_log(self, i, j, k, channel_data_start_index, channel_data_bytes, radius, reflectivity):
        logging.debug(f"datablock[{i}], firingsequence[{j}]; line[{k}]")
        logging.debug(f"-To process data start_index[{channel_data_start_index}], 3 bytes total")
        logging.debug(f"--channel_data_bytes = [{channel_data_bytes}]")
        logging.debug(f"radius = {radius}, reflectivity = {reflectivity}, ")

    def process_one_channel_data_return_radius_nd_reflectivity(self, bytes):
        assert len(bytearray(bytes)) == 3, "One channel data bytes must be a bytearray of length 3"

        # first two bytes are radius
        radius_in_two_bytes = self.calculate_distance_from_two_bytes(bytes[:2])  # first two bytes are radius
        reflectivity_in_one_bytes = self.calculate_reflection_ratio_from_one_byte(
            bytes[2:3])  # The third byte is reflectivity
        return radius_in_two_bytes, reflectivity_in_one_bytes

    def add_angle_increment(self):
        angle_list = []
        logging.debug(f"Default azimuth angle  with increment with {len(self.azimuth_angle_list)} items =")
        logging.debug(f"    {self.azimuth_angle_list}")
        for i, angle in enumerate(self.azimuth_angle_list):
            # append angle for first firing sequence in datablock
            angle_list.append(angle)

            # angle for second firing sequence = angle + angle_increment
            angle_second_firing_sequence = angle + self.azimuth_increment
            angle_second_firing_sequence = angle_second_firing_sequence % 360  # check if angle_second_firing_sequence is over 360 degree.

            angle_list.append(angle_second_firing_sequence)
        logging.debug(f"azimuth angle with increment with {len(angle_list)} items = ")
        logging.debug(f"    {angle_list}")
        return angle_list

    def calculate_default_aizmuth_angle_list(self):
        logging.info("Entered calculate_default_aizmuth_angle_list()")
        for i, block in enumerate(self.blocks):
            if convert_Bytearray_2_int(block[:2]) == 0xFFEE:  # Check if flag is correct, if not donot proceed;
                azimuth_angle = self.calculate_Azimuth_angle_from_2_bytes(block[2:4])
                logging.debug(f"raw_azimuth_angle{[i]} = {azimuth_angle}")
                self.azimuth_angle_list.append(azimuth_angle)
            else:
                logging.debug(f"block[{i}] flag 0xFFEE check failed.")

    def print_blocks_in_bytes(self):
        logging.debug("Entered print_blocks_in_bytes()")
        for i in range(len(self.blocks)):
            logging.debug(self.blocks[i])

    def get_return_mode(self):
        Dict_Velodyne_RETURN_MODE = {
            0x37: "Strongest",
            0x38: "Last Return",
            0x39: "Dual Return"
        }

        return_mode_int = self.blocks[-1][-2]
        logging.debug(Dict_Velodyne_RETURN_MODE[return_mode_int])

        return return_mode_int

    def calculate_azimuth_angle_list(self):
        Azimuth_INDEX_START = 2
        Azimuth_INDEX_END = 4
        for i in range(self.data_block_size):
            Azimuth_angle_first_half_data_block = self.calculate_Azimuth_angle_from_2_bytes(
                self.blocks[i][Azimuth_INDEX_START:Azimuth_INDEX_END])

            self.Azimuth_angle_list.append(Azimuth_angle_first_half_data_block)

            logging.debug(f"Azimuth_angle [{i} first half] = {Azimuth_angle_first_half_data_block} degree")

            Azimuth_angle_second_half_data_block = Azimuth_angle_first_half_data_block + self.azimuth_increment
            if Azimuth_angle_second_half_data_block > 360:
                Azimuth_angle_second_half_data_block = Azimuth_angle_second_half_data_block - 360
            self.Azimuth_angle_list.append(
                Azimuth_angle_second_half_data_block)  # each data block contains 2 firing sequence.

    def calculate_azimuth_angle_increment(self):
        if self.str_return_mode == "SINGLE_MODE":
            azimuth_angle_increment = self.total_azimuth_angle_increase / self.azimuth_increment_count_for_single_mode
        else:
            azimuth_angle_increment = self.total_azimuth_angle_increase / self.azimuth_increment_count_for_dual_mode
        logging.debug(f"azimuth_angle_increment = {azimuth_angle_increment} in UDP message")
        return azimuth_angle_increment

    def calculate_azimuth_angle_increase_in_one_data_bytes(self):
        Azimuth_INDEX_START = 2
        Azimuth_INDEX_END = 4

        logging.debug(f"self.data_block_size = {self.data_block_size}")

        raw_last_Azimuth_angle = self.blocks[self.data_block_size - 1][Azimuth_INDEX_START:Azimuth_INDEX_END]
        raw_first_Azimuth_angle = self.blocks[0][Azimuth_INDEX_START:Azimuth_INDEX_END]

        last_Azimuth_angle = self.calculate_Azimuth_angle_from_2_bytes(raw_last_Azimuth_angle)
        first_Azimuth_angle = self.calculate_Azimuth_angle_from_2_bytes(raw_first_Azimuth_angle)

        if last_Azimuth_angle < first_Azimuth_angle:
            last_Azimuth_angle = 360 + last_Azimuth_angle  # if turning past one circle,

        total_azimuth_increment = (last_Azimuth_angle - first_Azimuth_angle)
        logging.debug(f"Total azimuth increase is {total_azimuth_increment} degree.")
        return total_azimuth_increment

    def calculate_data_block_size(self):
        """
        Each data block starts with flag - 0xFFEE,
        check size base on flag,

        :return: size_int
        """
        data_block_size = 0
        for block in self.blocks:
            first_2_bytes_2_int = convert_Bytearray_2_int(block[:2])
            if first_2_bytes_2_int == 0xFFEE:
                data_block_size += 1
            else:
                break

        assert data_block_size == len(self.blocks) - 1, "Check block size failed."
        # one additional data block at the end of the message,
        # which has timestamp, return mode & sensor type.
        return data_block_size

    def calculate_Azimuth_angle_from_2_bytes(self, data_bytes):
        # Calculate Azimuth angle from raw data bytes
        # The input data_bytes is a bytearray of length 2.
        # THe output Azimuth_angle is in degree (°).
        MAX_AZIMUTH_ANGLE = 359.99
        # Check input validity
        assert len(data_bytes) == 2, "data_bytes must be a bytearray of length 2"

        # Calculate Azimuth angle
        raw_angle = int((data_bytes[1] << 8) + data_bytes[0])

        angle_in_hundredths_degrees = float(raw_angle) * 0.01
        Azimuth_angle = angle_in_hundredths_degrees

        # Check output validity
        assert 0 <= Azimuth_angle <= MAX_AZIMUTH_ANGLE, \
            f"Azimuth_angle must be within the range [0,{MAX_AZIMUTH_ANGLE}]"

        return Azimuth_angle

    def calculate_lidar_lines(self):
        Dict_Velodyne_SENSOR_Lines = {
            0x21: 32,
            0x22: 16,
            0x24: 16,
            0x28: 32,
            # 0x31: "TBD", // uncertain,
            0xA1: 128
        }
        product_ID = self.blocks[-1][-1]  # Consult product manual
        Lidar_Lines = Dict_Velodyne_SENSOR_Lines[product_ID]
        logging.debug(f"Lidar sensor has {Lidar_Lines} lines")
        return Lidar_Lines

    def calculate_distance_from_two_bytes(self, bytes):
        # Calculate distance from origin (0,0,0)
        # The input data_bytes is a bytearray of length 2.
        # THe output distance is in degree (m-meter).
        # if distance = 0 -> no measurable reflection OR not returned in time.

        assert len(bytes) == 2, "data_bytes must be a bytearray of length 2"
        # Calculate raw distance angle
        raw_distance = int((bytes[1] << 8) + bytes[0])
        distance = float(raw_distance) * 0.002
        return distance

    def calculate_reflection_ratio_from_one_byte(self, bytes):
        """
        Diffuse reflectors report values from 0 to 100 for reflectivities from 0% to 100%.
        Retroreflectors report values from 101 to 255, where 255 represents an ideal reflection.
        """
        assert len(bytearray(bytes)) == 1, "data_bytes must be a bytearray of length 1"

        raw_reflection = int.from_bytes(bytes, "big")
        reflection_ratio = float(raw_reflection / 100)
        return reflection_ratio

    def init_point_cloud_numpy(self):
        """
        :return: empty numpy array with possible MAX items possible.
        """
        # x,y,z, reflectivity, 4 items in total
        return np.empty((0, 4))

    # class Veldoyne_LiDAR_VLP16_LastReturn(Velodyne_LiDAR_Decode_Single_Return, Velodyne_LiDar_16_Lines):


#     def __init__(self, data_bytes):
#         super().__init__(data_bytes)
#         logging.debug("class <Veldoyne_LiDAR_VLP16_LastReturn> __init__ completed")
#
#
# class Veldoyne_LiDAR_VLP16_DualReturn(Velodyne_LiDAR_Decode_Dual_Return, Velodyne_LiDar_16_Lines):
#     def __init__(self, data_bytes):
#         super().__init__(data_bytes)
#         logging.debug("class <Veldoyne_LiDAR_VLP16_DualReturn> __init__ completed")
#
#     def calculate_azimuth_increment_count(self):
#         logging.debug("Entered calculate_azimuth_increment_count in class Veldoyne_LiDAR_VLP16_DualReturn")
#         return (self.data_block_size - 1)


# def generate_message_decoder(data_bytes):
#     # extract last 2 bytes from data
#     LAST_INDEX_FOR_MESSAGE = -2  # Second last = Return Mode; Last byte = Product ID
#     int_Lidar_sensor_ID_and_ReturnMode = convert_Bytearray_2_int(data_bytes[LAST_INDEX_FOR_MESSAGE:])
#     dict_Lidar_sensor_ID_and_ReturnMode = {
#         0x3722: Veldoyne_LiDAR_VLP16_Strongest,  # "Strongest for VLP-16 or PUCK-LITE"
#         0x3822: Veldoyne_LiDAR_VLP16_LastReturn,  # "Last Return for VLP-16 or PUCK-LITE"
#         0x3922: Veldoyne_LiDAR_VLP16_DualReturn,  # "Dual Return for VLP-16 or PUCK-LITE"
#     }
#     if int_Lidar_sensor_ID_and_ReturnMode in dict_Lidar_sensor_ID_and_ReturnMode:
#         lidar_decoder = dict_Lidar_sensor_ID_and_ReturnMode[int_Lidar_sensor_ID_and_ReturnMode]
#         logging.debug
#         ("Correct lidar message decoder is found.")
#         return lidar_decoder
#     else:
#         raise ValueError("Only VLP-16 is supported for now. "
#                          "You can update the code to accomplish your tasks.")

def convert_Spherial_2_Cartesian(radius, a_azimuth_angle_degree, W_elevation_angle_degree):
    logging.debug("Entered convert_Spherial_2_Cartesian()")

    # when point are too close, radius will be none;
    if radius == 0:
        logging.debug("Dummy point found;")
        return None, None, None
    else:
        # math sin cos, input is in radians, conversation is necessary
        a_azimuth_angle_radians = math.radians(a_azimuth_angle_degree)
        W_elevation_angle_radians = math.radians(W_elevation_angle_degree)

        # spherial to cartesian, from radius, a, w  -->   x,y,z
        x = radius * math.cos(W_elevation_angle_radians) * math.sin(a_azimuth_angle_radians)
        y = radius * math.cos(W_elevation_angle_radians) * math.cos(a_azimuth_angle_radians)
        z = radius * math.sin(W_elevation_angle_radians)
        logging.debug(f"Point found on [{x},{y},{z}]")
        return x, y, z



def init_logging_stdout(LOG_LEVEL):
    LOG_LEVEL = LOG_LEVEL
    logging.basicConfig(level=LOG_LEVEL, format='\033[2m%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d - %(message)s')

    logging.debug('Debug message')
    logging.info('Info message')
    logging.warning('Warning message')
    logging.error('Error message')
    logging.critical('Critical message')

def init_logging_log_file_out(LOG_LEVEL):
    LOG_LEVEL = LOG_LEVEL
    filename='python.log'
    filemode='a'
    logging.basicConfig(level=LOG_LEVEL, format='[%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d - %(message)s',filename = filename,filemode = filemode)
    logging.debug('Debug message')
    logging.info('Info message')
    logging.warning('Warning message')
    logging.error('Error message')
    logging.critical('Critical message')


def main():
    with open('Received_message_bytes_formate_NOT_covered.bin', 'rb') as f:
        data_bytes = f.read()
        logging.debug(f"Lidar data = {data_bytes}")
        # tmp_decoder = generate_message_decoder(data_bytes)
        # Lidar_Decoder = tmp_decoder(data_bytes)
        lidar_data_handler = Veldoyne_LiDAR_VLP16_Strongest(data_bytes)


if __name__ == "__main__":
    # LOG_LEVEL = logging.DEBUG
    LOG_LEVEL = logging.INFO


    # init_logging_stdout(LOG_LEVEL)
    init_logging_log_file_out(LOG_LEVEL)
    main()
