# Gemini 2.5 Pro

# audio.py
# An OSGAR driver for capturing audio using PyAudio.

import pyaudio
import threading
import time

from osgar.node import Node

# A mapping from string configuration to PyAudio format constants.
# This makes the JSON configuration more readable.
PYAUDIO_FORMAT_MAP = {
    'paInt16': pyaudio.paInt16,
    'paInt8': pyaudio.paInt8,
    'paInt32': pyaudio.paInt32,
    'paFloat32': pyaudio.paFloat32,
}

class Audio(Node):
    """
    OSGAR Node for capturing audio from a microphone using PyAudio.

    This node reads audio data from the specified input device and publishes
    it as raw bytes to the 'audio_data' output bus.

    Configuration:
    {
      "name": "audio",
      "driver": "app.audio:Audio",  // Replace with the actual path to this file
      "init": {
        "rate": 44100,              // Sample rate in Hz
        "channels": 1,              // Number of audio channels
        "format": "paInt16",        // Audio format (see PYAUDIO_FORMAT_MAP)
        "frames_per_buffer": 1024,  // Number of frames per buffer/chunk
        "device_index": null        // Optional: Specify input device index. If null, uses default.
      }
    }

    Outputs:
      - audio_data (bytes): Raw audio data chunks.
    """
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('audio_data:gz')

        # --- Configuration with Defaults ---
        self.rate = config.get('rate', 44100)
        self.channels = config.get('channels', 1)
        self.frames_per_buffer = config.get('frames_per_buffer', 1024)
        self.device_index = config.get('device_index')  # Can be None for default device

        # Get the PyAudio format constant from the string in the config.
        format_str = config.get('format', 'paInt16')
        if format_str not in PYAUDIO_FORMAT_MAP:
            raise ValueError(f"Unsupported audio format: {format_str}")
        self.format = PYAUDIO_FORMAT_MAP[format_str]

        # --- Internal State ---
        self.pyaudio = None
        self.stream = None
        self.thread = None  # The thread that will run the audio capture loop

    def start(self):
        """
        Initializes PyAudio and opens the audio stream.
        This method is called by OSGAR when the node is started.
        """
        self.pyaudio = pyaudio.PyAudio()
        try:
            self.stream = self.pyaudio.open(
                format=self.format,
                channels=self.channels,
                rate=self.rate,
                input=True,
                input_device_index=self.device_index,
                frames_per_buffer=self.frames_per_buffer
            )
        except Exception as e:
            print(f"Error opening PyAudio stream: {e}")
            # If the stream fails to open, we should not proceed.
            self.request_stop()
            return

        # We run the main loop in a separate thread to avoid blocking
        # the main OSGAR execution thread.
        self.thread = threading.Thread(target=self.run_capture_loop)
        super().start()  # trigger Node/Thread default start()
        self.thread.start()

    def run_capture_loop(self):
        """
        The main loop for capturing and publishing audio data.
        This runs in a separate thread.
        """
        print("Audio driver: Capture loop started.")
        self.stream.start_stream()

        # The try...finally block is crucial to ensure that resources are
        # released correctly, even if an error occurs.
        try:
            while self.is_alive():
                try:
                    # The read() call is blocking. It will wait until enough
                    # frames are available.
                    chunk = self.stream.read(self.frames_per_buffer, exception_on_overflow=False)
                    self.bus.publish('audio_data', chunk)
                except IOError as e:
                    # This can happen if the buffer overflows.
                    # We can log it and continue.
                    print(f"Audio capture IOError: {e}")

        finally:
            # --- Cleanup ---
            # This code is guaranteed to run when the loop exits.
            print("Audio driver: Stopping stream and terminating PyAudio.")
            if self.stream is not None and self.stream.is_active():
                self.stream.stop_stream()
            if self.stream is not None:
                self.stream.close()
            if self.pyaudio is not None:
                self.pyaudio.terminate()
            print("Audio driver: Cleanup complete.")


    def request_stop(self):
        """
        Called by OSGAR to signal that the node should stop.
        """
        # The is_alive() check in the run_capture_loop will fail,
        # causing the loop to terminate and the `finally` block to execute.
        super().request_stop()

    def join(self, timeout_sec=None):
        """
        Waits for the capture thread to finish.
        """
        if self.thread is not None:
            self.thread.join(timeout_sec)
        super().join(timeout_sec)

# --- Utility to List Audio Devices ---
# You can run this file directly (`python audio.py`) to list available
# audio devices and their indices, which you can then use in your JSON config.
if __name__ == "__main__":
    p = pyaudio.PyAudio()
    print("Available Audio Input Devices:")
    print("-" * 30)
    for i in range(p.get_device_count()):
        info = p.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            print(f"Index: {info['index']}, Name: {info['name']}, Channels: {info['maxInputChannels']}, Rate: {info['defaultSampleRate']}")
    print("-" * 30)
    p.terminate()
