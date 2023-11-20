import cv2
import cv2.aruco as aruco
import numpy as np

from opencv.webcam_input import WebcamFeed


class ArUcoMarkerDetector:
    """
        A class to detect ArUco markers in a webcam feed.

        Attributes
        ----------
        webcam_feed : WebcamFeed
            The WebcamFeed object to capture frames from.
        marker_dict : cv2.aruco_Dictionary
            The dictionary of ArUco markers to use. Defaults to aruco.DICT_6X6_250.
        ----------
        """

    def __init__(self,
                 webcam: WebcamFeed):
        print("Initializing aruco_detector...")

        self.webcam_feed = webcam
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        print("aruco_detector initialized.")

    def detect_markers(self):
        """
        Detect ArUco markers in the webcam feed. Display the frame with markers.
        :return: corners, ids, frame_markers --> The corners of the detected markers, the ids of the detected markers, and the frame with markers drawn on it.
        """
        frame = self.webcam_feed.single_capture_and_display()
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        parameters = aruco.DetectorParameters()

        parameters.minMarkerPerimeterRate = 0.01
        # corners go clockwise from top left
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.marker_dict, parameters=parameters)

        if corners != None and len(corners) > 0:
            print(corners)

        ids_to_direction = {}

        if ids is not None:
            corners = np.array(corners)
            ids = np.array(ids)
            centroids = []
            for id in range(len(ids)):
                top_left = corners[id][0][0]
                bottom_left = corners[id][0][3]
                top_middle = 1/2 * (corners[id][0][0] + corners[id][0][1])

                # Calculate centroid (middle) of the marker
                centroid = np.mean(corners[id][0], axis=0)
                cv2.circle(frame, tuple(map(int, centroid)), 5, (0, 255, 0), -1)

                direction = top_middle - centroid
                ids_to_direction[id] = direction

                cv2.arrowedLine(frame, tuple(map(int, centroid)),
                                tuple(map(int, top_middle)), (0, 255, 0), thickness=2, tipLength=0.1)

        # rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 100)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        # Display the frame with markers
        cv2.imshow('Aruco Marker Detection', frame_markers)

        # Press 'q' to exit the loop
        if self.webcam_feed.user_has_quit():
            self.webcam_feed.release_resources()

        return corners, ids, frame_markers, ids_to_direction

    def release_resources(self):
        self.webcam_feed.release_resources()


if __name__ == "__main__":
    # sample usage:
    webcam = WebcamFeed()
    aruco_detector = ArUcoMarkerDetector(webcam)

    while True:
        webcam.single_capture_and_display()

        if webcam.user_has_quit():
            webcam.release_resources()
            break

        corners, ids, frame_markers = aruco_detector.detect_markers()

    print("Done.")
