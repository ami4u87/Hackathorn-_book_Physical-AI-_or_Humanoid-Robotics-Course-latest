# Voice Command Interface

Enable natural voice interaction with your humanoid robot using speech-to-text and text-to-speech.

## Speech-to-Text Options

### Option 1: OpenAI Whisper (Recommended)

**Advantages**: Highly accurate, multilingual, open-source

```bash
pip install openai-whisper
```

```python
import whisper
import pyaudio
import wave
import numpy as np

class WhisperSTT:
    def __init__(self, model_size="base"):
        """
        Initialize Whisper STT

        Args:
            model_size: tiny, base, small, medium, large
        """
        self.model = whisper.load_model(model_size)

    def transcribe_file(self, audio_file):
        """Transcribe audio file"""
        result = self.model.transcribe(audio_file)
        return result['text']

    def transcribe_audio_data(self, audio_data, sample_rate=16000):
        """Transcribe numpy audio data"""
        # Save temporary file
        import tempfile
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            self.save_audio(audio_data, f.name, sample_rate)
            text = self.transcribe_file(f.name)

        return text

    def save_audio(self, data, filename, sample_rate):
        """Save audio data to WAV file"""
        import scipy.io.wavfile as wavfile
        wavfile.write(filename, sample_rate, data)
```

### Option 2: Google Speech Recognition

```bash
pip install SpeechRecognition
```

```python
import speech_recognition as sr

class GoogleSTT:
    def __init__(self):
        self.recognizer = sr.Recognizer()

    def listen_and_transcribe(self):
        """Listen to microphone and transcribe"""
        with sr.Microphone() as source:
            print("Listening...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            text = self.recognizer.recognize_google(audio)
            return text
        except sr.UnknownValueError:
            return None
        except sr.RequestError as e:
            print(f"Error: {e}")
            return None
```

## ROS 2 Voice Node

```python
from std_msgs.msg import String
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # STT engine
        self.stt = WhisperSTT(model_size="base")

        # Publisher
        self.command_pub = self.create_publisher(
            String, '/voice_command', 10
        )

        # Audio capture
        self.audio_buffer = []
        self.is_recording = False

        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def listen_loop(self):
        """Continuously listen for voice commands"""
        import pyaudio

        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000

        p = pyaudio.PyAudio()

        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK
        )

        print("Voice interface ready. Say 'robot' to start command...")

        while rclpy.ok():
            # Read audio chunk
            data = stream.read(CHUNK)
            audio_chunk = np.frombuffer(data, dtype=np.int16)

            # Detect speech (simple energy-based)
            if self.detect_speech(audio_chunk):
                self.record_command(stream, RATE)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def detect_speech(self, audio_chunk, threshold=500):
        """Simple voice activity detection"""
        energy = np.abs(audio_chunk).mean()
        return energy > threshold

    def record_command(self, stream, rate, max_duration=5):
        """Record voice command"""
        print("Recording...")

        frames = []
        for i in range(0, int(rate / 1024 * max_duration)):
            data = stream.read(1024)
            frames.append(np.frombuffer(data, dtype=np.int16))

            # Stop if silence detected
            if not self.detect_speech(frames[-1]):
                break

        # Convert to numpy array
        audio_data = np.concatenate(frames)

        # Transcribe
        text = self.stt.transcribe_audio_data(audio_data, rate)

        if text:
            print(f"Recognized: {text}")

            # Publish
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)
        else:
            print("No speech detected")
```

## Wake Word Detection

Add wake word to activate listening.

```python
import pv_porcupine

class WakeWordDetector:
    def __init__(self, wake_word="robot"):
        """
        Initialize wake word detector

        Requires Porcupine API key
        """
        self.porcupine = pvporcupine.create(
            keywords=[wake_word]
        )

    def detect_wake_word(self, audio_frame):
        """
        Check if wake word is in audio

        Args:
            audio_frame: 16-bit PCM audio frame

        Returns:
            True if wake word detected
        """
        keyword_index = self.porcupine.process(audio_frame)
        return keyword_index >= 0
```

## Text-to-Speech Feedback

Provide voice feedback to user.

```python
from gtts import gTTS
import os

class TextToSpeech:
    def __init__(self):
        pass

    def speak(self, text):
        """Convert text to speech and play"""
        tts = gTTS(text=text, lang='en')
        tts.save("response.mp3")

        # Play audio
        os.system("mpg123 response.mp3")

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.tts = TextToSpeech()

        # Subscribe to feedback messages
        self.feedback_sub = self.create_subscription(
            String, '/robot_feedback', self.feedback_callback, 10
        )

    def feedback_callback(self, msg):
        """Speak feedback message"""
        self.tts.speak(msg.data)
```

## Command Grammar

Define command patterns for better recognition.

```python
COMMAND_PATTERNS = {
    "pick_up": r"pick up (the )?(.*)",
    "place": r"place (it )?(on|in|at) (the )?(.*)",
    "move_to": r"move to (the )?(.*)",
    "look_at": r"look at (the )?(.*)",
    "give_me": r"give me (the )?(.*)"
}

import re

def parse_command(text):
    """Parse natural language command"""
    text = text.lower().strip()

    for intent, pattern in COMMAND_PATTERNS.items():
        match = re.match(pattern, text)
        if match:
            return {
                'intent': intent,
                'entities': match.groups()
            }

    return {'intent': 'unknown', 'entities': []}

# Example
result = parse_command("Pick up the red cup")
# {'intent': 'pick_up', 'entities': ('the ', 'red cup')}
```

## Complete Voice Pipeline

```python
class VoiceRobotInterface(Node):
    def __init__(self):
        super().__init__('voice_robot_interface')

        # Components
        self.stt = WhisperSTT()
        self.tts = TextToSpeech()

        # Publishers
        self.command_pub = self.create_publisher(String, '/task_command', 10)

        # Subscribers
        self.status_sub = self.create_subscription(
            String, '/task_status', self.status_callback, 10
        )

        self.current_status = "idle"

    def process_voice_input(self, audio_data):
        """Complete pipeline: voice → text → command"""

        # 1. Speech to text
        text = self.stt.transcribe_audio_data(audio_data)

        if not text:
            self.tts.speak("I didn't catch that. Please repeat.")
            return

        self.get_logger().info(f"Heard: {text}")
        self.tts.speak(f"You said: {text}")

        # 2. Parse command
        parsed = parse_command(text)

        if parsed['intent'] == 'unknown':
            self.tts.speak("I don't understand that command.")
            return

        # 3. Publish command
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

        self.tts.speak("Understood. Executing now.")

    def status_callback(self, msg):
        """React to task status"""
        status = msg.data

        if status == "COMPLETED" and self.current_status != "COMPLETED":
            self.tts.speak("Task completed successfully.")

        elif status == "FAILED" and self.current_status != "FAILED":
            self.tts.speak("Task failed. Please try again.")

        self.current_status = status
```

## Testing

```bash
# Test voice node
ros2 run voice_pkg voice_command_node

# Say: "Robot, pick up the red cup"

# Monitor published commands
ros2 topic echo /voice_command
```

## Further Reading

- [Whisper Documentation](https://github.com/openai/whisper)
- [SpeechRecognition Library](https://pypi.org/project/SpeechRecognition/)
