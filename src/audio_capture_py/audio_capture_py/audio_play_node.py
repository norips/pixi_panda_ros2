#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import sounddevice as sd
import numpy as np

from audio_common_msgs.msg import AudioData


class AudioPlaybackNode(Node):

    def __init__(self):
        super().__init__("audio_playback")

        self.declare_parameter("device", None)
        self.declare_parameter("sample_rate", 48000)
        self.declare_parameter("channels", [0,1,2,3])

        self.device = self.get_parameter("device").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value

        self.subscription = [
            self.create_subscription(
                AudioData,
                f"/audio{i}",
                self.audio_callback,
                10,
            ) 
            for i in self.channels
        ]

        self.stream = sd.OutputStream(
            device=self.device,
            samplerate=self.sample_rate,
            channels=len(self.channels),
            dtype=np.int16,
        )

        self.stream.start()

        self.get_logger().info("Playback started.")

    def audio_callback(self, msg):

        samples = np.frombuffer(msg.data, dtype=np.int16)

        samples = samples.reshape((-1, len(self.channels)))

        self.stream.write(samples)

    def destroy_node(self):

        self.stream.stop()
        self.stream.close()

        super().destroy_node()


def main():

    rclpy.init()

    node = AudioPlaybackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()