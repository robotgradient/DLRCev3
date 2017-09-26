from multiprocessing import Process, Pipe, Queue
from collections import namedtuple
import cv2
import time

class AsyncObjectDetector(Process):

    def __init__(self, camera, object_detector):
        super(AsyncObjectDetector, self).__init__()
        self.camera = camera
        self.object_detector = object_detector
        self.parent_pipe, self.child_pipe = Pipe()
        self.buffer = None
        self.start()


    def run(self):

        while True:
            frame = self.cap
            results = self.object_detector.detect(frame)
            self.child_pipe.send(results)

    @property
    def last_results(self):
        self.buffer =  self.parent_pipe.recv() if self.parent_pipe.poll() else self.buffer
        return self.buffer

    @property
    def frame(self):
        self.frame_buffer = self.parent_pipe.recv() if self.parent_pipe.poll() else self.frame_buffer
        return self.buffer

class AsyncCamera(Process):

    def __init__(self, cid):
        super(AsyncCamera, self).__init__()
        self.cid = cid
        self.img = None
        self.parent_pipe, self.child_pipe = Pipe()
        self.rpp, self.rpc = Pipe()
        self.buffer = None
        self.start()

        # Warmup
        counter = 0
        while self.frame is None:
            time.sleep(0.1)
            if counter % 100 == 0:
                print("Waiting for camera...")

    def run(self):
        print("Running process")
        cap = cv2.VideoCapture(self.cid)

        while True:
            t = time.time()
            self.img = cap.read()
            self.child_pipe.send(self.img)
    @property
    def frame(self):
        self.buffer = self.parent_pipe.recv() if self.parent_pipe.poll() else self.buffer
        return self.buffer

    def read(self):
        self.buffer = self.parent_pipe.recv() if self.parent_pipe.poll() else self.buffer
        return True, self.buffer

Circle = namedtuple("Circle", "x y r c")
BBox = namedtuple("BBox", "x1 y1 x2 y2 c")
Frame = namedtuple("Frame", "val")





class AsyncCameraRenderer(Process):

    def __init__(self, global_queue):
        super(AsyncCameraRenderer, self).__init__()
        self.global_queue = global_queue
        self.start()
        self.publishers = {}
        self.frame_buff = None

    def render(self, o):
        if isinstance(o, Circle):
            cv2.circle(self.old_frame, )
        elif isinstance(o, BBox):
            p1 = (int(o.x1), int(o.y1))
            p2 = (int(o.x2 ), int(o.y2))
            cv2.rectangle(self.frame_buff, p1, p2, (0, 0, 255))
    def run(self):
        while True:
            publisher, object = self.global_queue.get()
            self.publishers[publisher] = object
            for key, item in self.publishers.items():
                if isinstance(item, list):
                    for o in item:
                        self.render(o)
                else:
                    self.render(o)
            cv2.imshow("Camera1", self.frame_buff)


class AsyncObjectTracker(Process):

    def __init__(self, camera, tracker):
        super(AsyncObjectTracker, self).__init__()
        self.parent_pipe, self.child_pipe = Pipe()
        self.camera = camera
        self.tracker = tracker
        self.buffer = None
        self.frame_buffer = None
        self.rpp, self.rpc = Pipe()

        self.start()

    def run(self):

        while(True):
            res = self.camera.frame
            if res is None:
                continue
            else:
                ok, frame = res
            if self.child_pipe.poll():
                bbox = self.child_pipe.recv()
                tracker.init(frame, bbox)
            elif not frame is None:
                ok, bbox = tracker.update(frame)
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 0, 255))
                self.rpc.send(frame)

    def init(self, bbox):
        self.parent_pipe.send(bbox)

    @property
    def bbox(self):
        self.buffer = self.parent_pipe.recv() if self.parent_pipe.poll() else self.buffer
        return self.buffer

    @property
    def frame(self):
        self.frame_buffer = self.rpp.recv() if self.rpp.poll() else self.buffer
        return self.frame_buffer

