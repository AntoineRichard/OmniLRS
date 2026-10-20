import re
import threading
from collections import deque
from fractions import Fraction
from math import isfinite
from typing import Any, Dict

import av
import numpy as np
from PIL import Image

from src.subsystems.robot_physics_models.robot_physics_model import RobotPhysicsModel

# Annex B separates H.264 Network Abstraction Layer (NAL) units with three- or four-byte start codes.
START_CODE = b"\x00\x00\x00\x01"
ANNEX_B_START_CODE = re.compile(b"\x00\x00\x00\x01|\x00\x00\x01")


class CameraCompressionModel(RobotPhysicsModel):
    """Apply H.264 compression and simulated packet loss to sequential camera images.

    Uses target-bitrate mode with Video Buffering Verifier (VBV) constraints when `crf` is None.
    Setting `crf` selects quality-based mode and ignores `bitrate`.
    Resolution is fixed after the first frame.
    Keep one persistent instance per camera stream and call compute() once per new image.
    Timestamps advance at the configured fps; dt is unused.
    update_parameters() changes the compression settings and may be called from another thread (e.g. a telecommand);
    it waits for a running compute(). Packet loss (loss_probability, seed) is set only at initialize().

    Flow (after initialize()):
        [set_inputs(image)]
                | PIL image
                v
        +---------------------- compute() -----------------------+
        | [_initialize_codecs()]  (first frame only)             |
        |           |                                            |
        | [_encoder.encode(frame)]                               |
        |           | H.264 packets                              |
        |           v                                            |
        | +------------- _transmit_packet() -------------------+ |
        | | _split_nal_units()                                 | |
        | |        | NAL units                                 | |
        | |        v                                           | |
        | | split each NAL into network packets                | |
        | |        |                                           | |
        | |        v                                           | |
        | | drop each network packet with loss_probability     | |
        | |        | NALs with no lost network packet          | |
        | |        v                                           | |
        | | join surviving NALs into one H.264 packet          | |
        | +----------------------------------------------------+ |
        |           | surviving H.264 packets                    |
        |           v                                            |
        | [_decoder.decode(packet)]                              |
        |           | decoded image                              |
        |           v                                            |
        | [new image OR previous/black image if none decoded]    |
        +--------------------------------------------------------+
                |
                v
        [get_outputs()]
          compressed_image | bandwidth (bit/s) | packet/error counts

    x264 splits each frame into one slice per encoder thread, so slice count depends on the host CPU, and encoded
    bytes vary between runs; the seed only fixes which network packets drop for a given encoded stream.
    The threaded decoder does not conceal lost slices, which then decode as flat green (zero YUV).
    """

    def initialize(
        self,
        fps: float = 1,  # Frame arrival rate; sets timestamps and the bitrate budget.
        bitrate: int = 20_000,  # Target bits/s; used only when crf is None.
        keyframe_interval: int = 10,  # Group of pictures size: number of frames between keyframes.
        crf: int | None = None,  # Set 0-51 for quality-based mode; when set, bitrate is ignored.
        max_payload_size: int = 1200,  # Maximum NAL payload bytes per network packet.
        loss_probability: float = 0.01,  # Independent probability of dropping each network packet.
        seed: int | None = None,  # Seed for reproducible packet loss.
        bandwidth_window_seconds: float = 10.0,  # Bitrate averaging window, rounded to whole frames (at least one).
    ) -> None:
        if fps <= 0:
            raise ValueError("fps must be positive")
        if not 0 <= loss_probability <= 1:
            raise ValueError("loss_probability must be between 0 and 1")
        if seed is not None and seed < 0:
            raise ValueError("seed must be non-negative, or None")
        self._validate_parameters(bitrate, keyframe_interval, crf, max_payload_size, bandwidth_window_seconds)

        self._image: Image.Image | None = None
        self._compressed_image: Image.Image | None = None
        self._resolution: tuple[int, int] | None = None
        self._fps = fps
        # Link conditions are fixed by the configuration; operators cannot command them.
        self._loss_probability = loss_probability
        self._rng = np.random.default_rng(seed)
        self._lock = threading.Lock()
        self._frame_index = 0
        self._encoded_frame_bytes: deque[int] = deque()  # Encoded bytes of each recent frame, oldest first.
        self._packets_sent = 0
        self._packets_lost = 0
        self._decode_errors = 0
        self._last_frame: Image.Image | None = None
        # Persistent H.264 decoder
        self._decoder = av.CodecContext.create("h264", "r")
        self._decoder.open()
        self._apply_parameters(bitrate, keyframe_interval, crf, max_payload_size, bandwidth_window_seconds)

    def update_parameters(
        self,
        bitrate: int,
        keyframe_interval: int,
        crf: int | None,
        max_payload_size: int,
        bandwidth_window_seconds: float,
    ) -> None:
        """Set the operator-commandable compression parameters described in initialize().

        The encoder is rebuilt on the next compute() so bitrate, crf and keyframe_interval apply; the decoder is kept
        and resynchronizes at the new keyframe. Packet loss settings are fixed at initialize().
        """
        self._validate_parameters(bitrate, keyframe_interval, crf, max_payload_size, bandwidth_window_seconds)
        self._apply_parameters(bitrate, keyframe_interval, crf, max_payload_size, bandwidth_window_seconds)

    @staticmethod
    def _validate_parameters(
        bitrate: int,
        keyframe_interval: int,
        crf: int | None,
        max_payload_size: int,
        bandwidth_window_seconds: float,
    ) -> None:
        # CRF: 0-51, lower means higher quality. None uses bitrate instead.
        if crf is not None and not 0 <= crf <= 51:
            raise ValueError("crf must be between 0 and 51, or None")
        if keyframe_interval <= 0:
            raise ValueError("keyframe_interval must be positive")
        if bitrate <= 0:
            raise ValueError("bitrate must be positive")
        if crf is None and bitrate < 1000:
            raise ValueError("Video Buffering Verifier bitrate mode requires at least 1000 bits/s")
        if max_payload_size <= 0:
            raise ValueError("max_payload_size must be positive")
        if not isfinite(bandwidth_window_seconds) or bandwidth_window_seconds <= 0:
            raise ValueError("bandwidth_window_seconds must be a finite positive number")

    def _apply_parameters(
        self,
        bitrate: int,
        keyframe_interval: int,
        crf: int | None,
        max_payload_size: int,
        bandwidth_window_seconds: float,
    ) -> None:
        with self._lock:
            self._bitrate = bitrate
            self._keyframe_interval = keyframe_interval
            self._crf = crf
            self._max_payload_size = max_payload_size
            # Keep one size per frame for window * fps frames; appending a new frame then drops the oldest.
            # TODO: no upper bound on bandwidth_window_seconds (none in the Yamcs MDB either). A huge value raises
            # OverflowError here, which the commander does not catch; add a maximum in the MDB or validate it above.
            window_frames = max(1, round(bandwidth_window_seconds * self._fps))
            self._encoded_frame_bytes = deque(self._encoded_frame_bytes, maxlen=window_frames)
            self._encoder = None

    def set_inputs(self, image: Image.Image | None) -> None:
        self._image = image

    def compute(self, dt: float) -> None:
        with self._lock:
            self._compute()

    def _compute(self) -> None:
        if self._image is None:
            self._compressed_image = None
            return

        if self._encoder is None:
            self._initialize_codecs(self._image.size)
        elif self._image.size != self._resolution:
            raise ValueError("Image resolution cannot change after H.264 encoding starts")

        image_rgb = np.asarray(self._image.convert("RGB"))
        # Convert NumPy RGB image to an FFmpeg frame.
        frame = av.VideoFrame.from_ndarray(image_rgb, format="rgb24")
        frame.pts = self._frame_index
        frame.time_base = self._encoder.time_base
        self._frame_index += 1

        # Keep the encoder and decoder alive across frames to preserve H.264 references.
        decoded_frames = []
        frame_bytes = 0
        for packet in self._encoder.encode(frame):
            frame_bytes += packet.size
            received_packet = self._transmit_packet(packet)
            if received_packet is None:
                continue
            try:
                decoded_frames.extend(self._decoder.decode(received_packet))
            except av.error.InvalidDataError:
                # Preserve decoder state; a later keyframe can restore the stream.
                self._decode_errors += 1

        self._encoded_frame_bytes.append(frame_bytes)

        if decoded_frames:
            decoded_rgb = decoded_frames[-1].to_ndarray(format="rgb24")
            self._compressed_image = Image.fromarray(decoded_rgb)
            self._last_frame = self._compressed_image.copy()
        elif self._last_frame is not None:
            # Freeze the previous image when no new frame can be decoded.
            self._compressed_image = self._last_frame.copy()
        else:
            # No decoded frame has arrived yet.
            self._compressed_image = Image.new("RGB", self._image.size, color=(0, 0, 0))

    def _transmit_packet(self, packet: av.Packet) -> av.Packet | None:
        """Send one encoded H.264 packet over a simulated lossy link and return what the receiver can decode.

        Each NAL unit is split into network packets of at most max_payload_size bytes, and each network packet is
        dropped independently with loss_probability. A NAL can only be rebuilt if all its network packets arrive, so
        one lost network packet discards the whole NAL while the other NALs still reach the decoder.
        Since a lost network packet discards its NAL anyway, network packets are only counted, not built as objects;
        the result is the same as splitting the payload, dropping some pieces and reassembling.
        """
        nal_units = self._split_nal_units(bytes(packet))

        received_nal_units = []
        for nal in nal_units:
            # Ceiling division: the last network packet may be partially filled.
            network_packet_count = (len(nal) + self._max_payload_size - 1) // self._max_payload_size
            # One random draw per network packet, in transmission order.
            lost_network_packets = sum(self._rng.random() < self._loss_probability for _ in range(network_packet_count))
            self._packets_sent += network_packet_count
            self._packets_lost += lost_network_packets
            if lost_network_packets == 0:
                received_nal_units.append(nal)
        if not received_nal_units:
            return None

        # Rebuilt packets retain the original frame timing for the decoder.
        received_packet = av.Packet(b"".join(START_CODE + nal for nal in received_nal_units))
        received_packet.pts = packet.pts
        received_packet.dts = packet.dts
        if packet.time_base is not None:
            received_packet.time_base = packet.time_base
        return received_packet

    @staticmethod
    def _split_nal_units(data: bytes) -> list[bytes]:
        """Extract Network Abstraction Layer (NAL) units from a complete Annex B packet."""
        if not data:
            return []

        start_codes = list(ANNEX_B_START_CODE.finditer(data))
        if not start_codes:
            raise ValueError("Encoded packet does not contain Annex B start codes")
        if any(data[: start_codes[0].start()]):
            raise ValueError("Unexpected data before the first Annex B start code")

        nal_units = []
        for index, start_code in enumerate(start_codes):
            end = start_codes[index + 1].start() if index + 1 < len(start_codes) else len(data)
            # Start codes and trailing padding are not part of the NAL payload.
            nal = data[start_code.end() : end].rstrip(b"\x00")
            if nal:
                nal_units.append(nal)
        return nal_units

    def _initialize_codecs(self, resolution: tuple[int, int]) -> None:
        width, height = resolution
        # yuv420p subsamples chroma 2x2, so it needs even width and height; yuv444p keeps full chroma.
        pixel_format = "yuv420p" if width % 2 == 0 and height % 2 == 0 else "yuv444p"

        # Persistent H.264 encoder
        self._encoder = av.CodecContext.create("libx264", "w")
        self._encoder.width = width
        self._encoder.height = height
        self._encoder.pix_fmt = pixel_format
        self._encoder.framerate = Fraction(self._fps).limit_denominator(1000)
        self._encoder.time_base = 1 / self._encoder.framerate
        if self._crf is None:
            self._encoder.bit_rate = self._bitrate
        self._encoder.gop_size = self._keyframe_interval
        self._encoder.max_b_frames = 0  # Avoid frame reordering and its extra latency.
        self._encoder.options = {
            "preset": "veryfast",  # Encoding speed/efficiency tradeoff, slower presets give better compression.
            "tune": "zerolatency",  # Minimize latency for real-time applications.
            "x264-params": (
                f"keyint={self._keyframe_interval}:"  # Maximum interval between keyframes.
                f"min-keyint={self._keyframe_interval}:"  # Minimum interval between keyframes.
                "scenecut=0:"
                "repeat-headers=1:"  # Repeat decoder settings at keyframes for recovery after loss.
                "annexb=1"
            ),
        }
        if self._crf is None:
            self._encoder.options["x264-params"] += (
                f":vbv-maxrate={self._bitrate // 1000}:vbv-bufsize={self._bitrate // 1000}"
            )
        else:
            # Quality mode: bitrate and its Video Buffering Verifier limits are not applied.
            self._encoder.options["crf"] = str(self._crf)
        self._encoder.open()
        self._resolution = resolution

    @property
    def bandwidth(self) -> float:
        """Average encoded H.264 bitrate in bits per second (bit/s) over the last `bandwidth_window_seconds`.

        Sums the encoded size of the frames in the window and divides by the seconds they cover. During startup,
        fewer frames are available and the average is over the time elapsed so far. The size is measured before
        simulated packet loss and includes Annex B start codes but not transport overhead, so it is not the bitrate
        received by the decoder.
        """
        if not self._encoded_frame_bytes:
            return 0.0
        window_seconds = len(self._encoded_frame_bytes) / self._fps
        return sum(self._encoded_frame_bytes) * 8 / window_seconds

    def get_outputs(self) -> Dict[str, Any]:
        """Return the current image, rolling average bandwidth in bits per second, and cumulative transport counters."""
        return {
            "compressed_image": self._compressed_image,
            "bandwidth": self.bandwidth,  # bits per second (bit/s)
            "packets_sent": self._packets_sent,
            "packets_lost": self._packets_lost,
            "decode_errors": self._decode_errors,
        }
