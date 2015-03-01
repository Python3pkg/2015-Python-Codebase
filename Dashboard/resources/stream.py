__author__ = 'Developer'

import socket
import time
import picamera

def main():
    camera = picamera.PiCamera()
    try:
        server_socket = socket.socket()
        server_socket.bind(('0.0.0.0',8000))
        server_socket.listen(0)

        stream = server_socket.accept()[0].makefile('wb')
        camera.resolution = (640, 480)
        camera.framerate = 12
        camera.start_preview()
        time.sleep(2)
        camera.stop_preview()
        print('ready to stream')
        time.sleep(.1)
        camera.start_recording(stream, format= 'h264')
        camera.wait_recording(60)
    except Exception as e:
        print(e)
    finally:
        camera.stop_recording()
        stream.close()
        server_socket.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

