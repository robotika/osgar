# Gemini 2.5 Pro

# log_to_wav.py
# A utility to extract audio data from an OSGAR log file and save it as a WAV file.
#
# This script reads an OSGAR log, collects all data chunks from a specified
# audio stream, and writes them into a standard WAV file.
#
# Usage:
# python log_to_wav.py <path_to_log_file> <output_wav_file> --stream-name audio.audio_data
#
# You must provide the audio parameters that were used during the recording.

import argparse
import wave
from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

# PyAudio format from the driver maps to sample width in bytes for the wave module.
# For example, 'paInt16' is 16 bits, which is 2 bytes.
FORMAT_TO_SAMPLE_WIDTH = {
    'paInt16': 2,
    'paInt8': 1,
    'paInt32': 4,
    # Note: The 'wave' module does not natively support float formats.
    # If you record in paFloat32, you would need a library like scipy or numpy
    # to convert the data to an integer format before saving.
}

def extract_audio_to_wav(log_path, wav_path, stream_name, rate, channels, audio_format):
    """
    Reads an OSGAR log, extracts audio data, and saves it to a WAV file.

    :param log_path: Path to the OSGAR .log file.
    :param wav_path: Path for the output .wav file.
    :param stream_name: The name of the audio stream in the log (e.g., 'audio.audio_data').
    :param rate: The sample rate of the audio (e.g., 44100).
    :param channels: The number of audio channels (e.g., 1 for mono).
    :param audio_format: The PyAudio format string used during recording (e.g., 'paInt16').
    """
    if audio_format not in FORMAT_TO_SAMPLE_WIDTH:
        print(f"Error: Unsupported or invalid audio format '{audio_format}'.")
        print("Supported formats for WAV conversion: 'paInt16', 'paInt8', 'paInt32'")
        return

    sample_width = FORMAT_TO_SAMPLE_WIDTH[audio_format]
    print(f"Reading log: {log_path}")
    print(f"Looking for stream: {stream_name}")

    audio_chunks = []

    try:
        # First, get the stream ID using the standalone utility function.
        # This will raise ValueError if the stream is not found.
        stream_id = lookup_stream_id(log_path, stream_name)
        
        # Now, use the LogReader with the 'only_stream_id' parameter for efficiency.
        # This makes the reader only yield data from our desired stream.
        with LogReader(log_path, only_stream_id=stream_id) as log:
            for timestamp, stream, data in log:
                # The raw data from the bus is serialized, so we need to deserialize it.
                # For simple bytes, this just unwraps it.
                chunk = deserialize(data)
                audio_chunks.append(chunk)

    except ValueError:
        print(f"Error: Stream '{stream_name}' not found in the log file '{log_path}'.")
        return
    except Exception as e:
        print(f"An error occurred while reading the log file: {e}")
        return

    if not audio_chunks:
        print(f"No data found for stream '{stream_name}'. WAV file not created.")
        return

    print(f"Found {len(audio_chunks)} audio chunks.")
    
    # Concatenate all the raw byte chunks into a single byte string
    all_audio_data = b''.join(audio_chunks)

    # Write the collected audio data to a WAV file
    try:
        with wave.open(wav_path, 'wb') as wf:
            wf.setnchannels(channels)
            wf.setsampwidth(sample_width)
            wf.setframerate(rate)
            wf.writeframes(all_audio_data)
        print(f"Successfully created WAV file: {wav_path}")
    except Exception as e:
        print(f"An error occurred while writing the WAV file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract audio from an OSGAR log file to a WAV file.")
    parser.add_argument("log_path", help="Path to the OSGAR log file.")
    parser.add_argument("wav_path", help="Path to the output WAV file.")
    parser.add_argument("--stream-name", default="audio.audio_data", help="Name of the audio stream in the log.")
    parser.add_argument("--rate", type=int, default=44100, help="Sample rate of the audio.")
    parser.add_argument("--channels", type=int, default=1, help="Number of audio channels.")
    parser.add_argument("--format", default="paInt16", choices=FORMAT_TO_SAMPLE_WIDTH.keys(),
                        help="Audio format used during recording.")

    args = parser.parse_args()

    extract_audio_to_wav(
        log_path=args.log_path,
        wav_path=args.wav_path,
        stream_name=args.stream_name,
        rate=args.rate,
        channels=args.channels,
        audio_format=args.format
    )
