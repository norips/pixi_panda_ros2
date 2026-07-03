#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sounddevice as sd
import numpy as np

from audio_common_msgs.msg import AudioData


class AudioCaptureNode(Node):

    def __init__(self):
        super().__init__("audio_capture")

        # Parameters
        self.declare_parameter("device", "H5")
        self.declare_parameter("sample_rate", 48000)
        self.declare_parameter("channels", 4)
        self.declare_parameter("block_size", 1024)

        self.device = self.get_parameter("device").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value
        self.block_size = self.get_parameter("block_size").value

        self.publisher = [self.create_publisher(AudioData, f"/audio{i}", 10) for i in range(self.channels)] 

        self.get_logger().info("Available audio devices:")
        print(sd.query_devices())

        self.get_logger().info(
            f"Opening device '{self.device}' "
            f"({self.channels} channels @ {self.sample_rate} Hz)"
        )

        self.stream = sd.InputStream(
            device=self.device,
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype=np.int16,
            blocksize=self.block_size,
            callback=self.audio_callback,
        )

        self.stream.start()

    def audio_callback(self, indata, frames, time, status):

        if status:
            self.get_logger().warning(str(status))

        # indata.shape = (block_size, channels)
        self.get_logger().debug(f"Received block {indata.shape}")

        for i in range(self.channels):
            msg = AudioData()
            msg.data = indata[:,i].tobytes()
            self.publisher[i].publish(msg)

    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()


def main():

    rclpy.init()

    node = AudioCaptureNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()