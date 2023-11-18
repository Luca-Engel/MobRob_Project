import cv2
import cv2.aruco as aruco

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
        frame = self.webcam_feed.single_capture_and_display()
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        parameters = aruco.DetectorParameters()

        parameters.minMarkerPerimeterRate = 0.01
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.marker_dict, parameters=parameters)

        # Draw markers on the frame
        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        # Display the frame with markers
        cv2.imshow('Aruco Marker Detection', frame_markers)

        # Press 'q' to exit the loop
        if self.webcam_feed.user_has_quit():
            self.webcam_feed.release_resources()

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

        aruco_detector.detect_markers()

    print("Done.")
