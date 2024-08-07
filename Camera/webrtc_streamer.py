import asyncio
import json
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.contrib.signaling import TcpSocketSignaling
from av import VideoFrame

class CameraVideoStreamTrack(VideoStreamTrack):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise IOError("Cannot open video device")
        print("Camera opened successfully")

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        ret, frame = self.cap.read()
        if not ret:
            raise IOError("Failed to read frame from video device")

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = pts
        video_frame.time_base = time_base
        return video_frame

    def stop(self):
        self.cap.release()
        print("Camera released")

async def run(pc, signaling):
    await signaling.connect()

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        print(f"ICE connection state is {pc.iceConnectionState}")
        if pc.iceConnectionState == "failed":
            await pc.close()
            await signaling.close()

    @pc.on("icecandidate")
    async def on_icecandidate(event):
        if event.candidate:
            candidate = event.candidate.to_json()
            print(f"Sending ICE candidate: {candidate}")
            await signaling.send(json.dumps({"candidate": candidate}))

    # Add the video track
    video_track = CameraVideoStreamTrack()
    pc.addTrack(video_track)

    while True:
        obj = await signaling.receive()
        if obj is None:
            continue
        print(f"Received signaling message: {obj}")
        if "sdp" in obj:
            print(f"Received SDP: {obj['sdp']['type']}")
            sdp = RTCSessionDescription(obj["sdp"]["sdp"], obj["sdp"]["type"])
            await pc.setRemoteDescription(sdp)
            if sdp.type == "offer":
                await pc.setLocalDescription(await pc.createAnswer())
                await signaling.send(json.dumps({"sdp": pc.localDescription.to_json()}))
                print("Sent SDP answer")
        elif "candidate" in obj:
            candidate = obj["candidate"]
            print(f"Received ICE candidate: {candidate}")
            await pc.addIceCandidate(RTCIceCandidate(candidate))

    video_track.stop()

if __name__ == "__main__":
    signaling = TcpSocketSignaling("192.168.31.115", 9000)  # Replace with the correct IP address of your Raspberry Pi
    pc = RTCPeerConnection()

    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(pc.close())
        loop.run_until_complete(signaling.close())
