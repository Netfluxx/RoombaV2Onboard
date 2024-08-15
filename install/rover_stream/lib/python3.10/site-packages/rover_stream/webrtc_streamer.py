#!/usr/bin/env python3

import asyncio
import rclpy
from rclpy.node import Node
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.signaling import TcpSocketSignaling
import cv2
from av import VideoFrame
import json

BYE = "BYE"

class CameraVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            raise IOError("Cannot open video device")
        print("Camera opened successfully")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret:
            raise IOError("Failed to read frame from video device")

        yuv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        video_frame = VideoFrame.from_ndarray(yuv_frame, format="yuv420p")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        self.cap.release()
        print("Camera released")

class WebRTCStreamer(Node):
    def __init__(self):
        super().__init__('webrtc_streamer')
        self.declare_parameter('signaling_host', '192.168.31.158')  # Replace with your laptop's IP address
        self.declare_parameter('signaling_port', 8000)
        self.pc = RTCPeerConnection()
        self.signaling_host = self.get_parameter('signaling_host').get_parameter_value().string_value
        self.signaling_port = self.get_parameter('signaling_port').get_parameter_value().integer_value
        self.signaling = TcpSocketSignaling(self.signaling_host, self.signaling_port)
        self.video_track = CameraVideoStreamTrack()
        self.pc.addTrack(self.video_track)

        @self.pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            print(f"ICE gathering state changed: {self.pc.iceGatheringState}")

        @self.pc.on("icecandidate")
        async def on_icecandidate(event):
            print(f"ICE candidate: {event.candidate}")
            if event.candidate:
                await self.signaling.send(json.dumps({'ice': event.candidate.to_json()}))
                print(f"Sent ICE candidate: {event.candidate}")

        @self.pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print(f"ICE connection state: {self.pc.iceConnectionState}")
            if self.pc.iceConnectionState == "failed":
                await self.pc.close()
                await self.signaling.close()

    async def run(self):
        print(f"Connecting to signaling server at {self.signaling_host}:{self.signaling_port}")
        await self.signaling.connect()

        while True:
            message = await self.signaling.receive()
            if message is not None:
                try:
                    obj = json.loads(message)
                    print(f"Received signaling message: {obj}")
                    if 'sdp' in obj:
                        sdp = RTCSessionDescription(**obj['sdp'])
                        await self.pc.setRemoteDescription(sdp)
                        if sdp.type == "offer":
                            await self.pc.setLocalDescription(await self.pc.createAnswer())
                            await self.signaling.send(json.dumps({'sdp': self.pc.localDescription.to_json()}))
                            print("Sent SDP answer")
                    elif 'ice' in obj:
                        ice = obj['ice']
                        candidate = RTCIceCandidate(
                            component=ice['component'],
                            foundation=ice['foundation'],
                            ip=ice['ip'],
                            port=ice['port'],
                            priority=ice['priority'],
                            protocol=ice['protocol'],
                            type=ice['type'],
                            sdpMid=ice['sdpMid'],
                            sdpMLineIndex=ice['sdpMLineIndex']
                        )
                        await self.pc.addIceCandidate(candidate)
                except json.JSONDecodeError:
                    print("Received non-JSON message")
            else:
                print("Received empty message")

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCStreamer()
    try:
        asyncio.run(node.run())
    except KeyboardInterrupt:
        pass
    finally:
        node.video_track.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
