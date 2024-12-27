import cv2
import asyncio
import json
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCIceCandidate
from aiortc.contrib.signaling import BYE
import websockets
from aiortc.contrib.media import MediaStreamTrack
from datetime import datetime
from av import VideoFrame
import av
import fractions
import sys
import subprocess
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
latest_frame = None

async def signaling(websocket, pc):
    while True:
        message = await websocket.recv()
        print(f"Received message: {message}")  # Print message when received

        if message == BYE:
            print("Received BYE, stopping")
            await pc.close()
            break
        else:
            msg = json.loads(message)

            if 'candidate' in msg:
                candidate_parts = msg['candidate'].split(' ')

                # Print each part of the candidate for debugging
                print("Split candidate parts:")
                for index, part in enumerate(candidate_parts):
                    print(f"Part {index}: {part}")

                ice_candidate = RTCIceCandidate(
                    foundation=candidate_parts[0].split(':')[1],  # Removing 'candidate:' prefix
                    component=int(candidate_parts[1]),             # Component
                    protocol=candidate_parts[2],                   # Protocol
                    priority=int(candidate_parts[3]),              # Priority
                    ip=candidate_parts[4],                         # IP Address
                    port=int(candidate_parts[5]),                  # Port
                    type=candidate_parts[7],                       # Candidate Type
                    sdpMid=msg.get('sdpMid', None),                # SDP Mid
                    sdpMLineIndex=msg.get('sdpMLineIndex', None)   # SDP MLine Index
                )

                await pc.addIceCandidate(ice_candidate)

                print("after add ice candidate ")
            elif 'sdp' in msg:
                # Check if msg['type'] is 2 or '2' and convert it to "answer"
                if msg['type'] == 2 or msg['type'] == '2':
                    msg['type'] = "answer"
                
                desc = RTCSessionDescription(sdp=msg['sdp'], type=msg['type'])
                await pc.setRemoteDescription(desc)

                if desc.type == "offer":
                    await pc.setLocalDescription(await pc.createAnswer())
                    await websocket.send(json.dumps({
                        'sdp': pc.localDescription.sdp,
                        'type': pc.localDescription.type
                    }))

async def create_offer(websocket, pc):
    # Print the timestamp when create_offer is called
    print(f"create_offer called at {datetime.now().isoformat()}")
    
    await pc.setLocalDescription(await pc.createOffer())
    await websocket.send(json.dumps({
        'sdp': pc.localDescription.sdp,
        'type': pc.localDescription.type
    }))

class CameraVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, device_index=0, fps=30):
        super().__init__()
        self.cap = cv2.VideoCapture(device_index)
        self.fps = fps
        self.frame_counter = 0
        self.time_base = fractions.Fraction(1, self.fps)
        self.count = 0  # Initialize count

        if not self.cap.isOpened():
            raise Exception("Could not open video device")

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture video frame")

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        new_frame = VideoFrame.from_ndarray(frame, format="rgb24")

        # Calculate pts manually
        pts = self.frame_counter * self.time_base.denominator
        self.frame_counter += 1

        new_frame.pts = pts
        new_frame.time_base = self.time_base

        # Increment count and check if it exceeds 30
        self.count += 1
        if self.count > 30:
            print("Count has exceeded 30, resetting count to 0")
            self.count = 0

        return new_frame

    def stop(self):
        self.cap.release()
        super().stop()

def list_gstreamer_cameras():
    devices = []
    command = 'v4l2-ctl --list-devices'
    result = subprocess.run(command, shell=True, stdout=subprocess.PIPE)
    result_text = result.stdout.decode('utf-8')
    
    # Parsing the output
    for line in result_text.split('\n'):
        if '/dev/video' in line:
            devices.append(line.strip())
    
    return devices

class ImageVideoTrack(VideoStreamTrack):
    async def recv(self):
        global latest_frame
        pts, time_base = await self.next_timestamp()

        # Convert ROS image to OpenCV format
        if latest_frame is not None:
            frame = av.VideoFrame.from_ndarray(latest_frame, format='bgr24')
            frame.pts = pts
            frame.time_base = time_base
            return frame
        else:
            return None

def image_callback(ros_image):
    global latest_frame
    # Convert ROS Image message to OpenCV image
    latest_frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    
async def main():
    if len(sys.argv) < 2:
        print("Usage: python client.py <server_ip>")
        return

    server_ip = sys.argv[1]

    rospy.init_node('leju_image_to_webrtc')
    rospy.Subscriber("/image_publisher_1723543173973916584/camera_info", Image, image_callback)

    uri = f"ws://{server_ip}:8765"

    pc = RTCPeerConnection()

    available_cameras = list_gstreamer_cameras()
    print("Available camera devices:", available_cameras)

    if available_cameras:
        video_track = CameraVideoTrack(device_index=available_cameras[0])
    else:
        print("No available camera devices found.")
        return

    pc.addTrack(video_track)
    print("Video track has been added to the peer connection.")  # Debugging statement

    async with websockets.connect(uri) as websocket:
        await websocket.send("client1")
        await create_offer(websocket, pc)
        await signaling(websocket, pc)

    await pc.close()


if __name__ == "__main__":
    asyncio.run(main())
